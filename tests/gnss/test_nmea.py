"""NMEA validation tests; all positions here are fixtures, never training/control data."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "gnss"))
from nmea import MAX_SENTENCE_LENGTH, parse_sentence


def sentence(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f"${body}*{checksum:02X}"


GGA = "GNGGA,040203.125,0955.1234567,N,08405.7654321,W,4,28,0.6,100.25,M,-12.50,M,1.2,0123"
RMC = "GNRMC,040203.125,A,0955.1234567,N,08405.7654321,W,1.25,123.4,301026,,,R"


class NmeaTests(unittest.TestCase):
    def test_independently_known_checksum_example(self):
        value = parse_sentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n")
        self.assertAlmostEqual(value["latitude_deg"], 48.1173)
        self.assertAlmostEqual(value["longitude_deg"], 11.516666666666667)
        self.assertAlmostEqual(value["altitude_ellipsoid_m"], 592.3)
        self.assertEqual(value["message_type"], "GGA")
        self.assertEqual(value["talker"], "GP")

    def test_fixed_heights_sign_time_and_no_invented_date(self):
        value = parse_sentence(sentence(GGA))
        self.assertEqual(value["fix_quality"], 4)
        self.assertEqual(value["fix_type"], "rtk_fixed")
        self.assertTrue(value["position_valid"])
        self.assertEqual(value["satellites_used"], 28)
        self.assertEqual(value["utc_seconds_of_day"], 14523.125)
        self.assertEqual(value["utc_time"], "04:02:03.125")
        self.assertEqual(value["altitude_msl_m"], 100.25)
        self.assertEqual(value["geoid_separation_m"], -12.5)
        self.assertEqual(value["altitude_ellipsoid_m"], 87.75)
        self.assertLess(value["longitude_deg"], 0)
        self.assertEqual(value["correction_age_s"], 1.2)
        self.assertEqual(value["station_id"], "0123")
        self.assertNotIn("utc_date", value)
        self.assertFalse(any("receipt" in field or "unix" in field for field in value))

    def test_fix_quality_not_replaced_by_coordinate_presence(self):
        for quality, label in ((0, "invalid"), (4, "rtk_fixed"), (5, "rtk_float"), (6, "dead_reckoning")):
            with self.subTest(quality=quality):
                value = parse_sentence(sentence(GGA.replace(",4,28,", f",{quality},28,")))
                self.assertEqual(value["fix_type"], label)
                self.assertEqual(value["position_valid"], quality != 0)

    def test_no_fix_and_null_data_remain_unknown(self):
        value = parse_sentence(sentence("GPGGA,,,,,,0,00,99.99,,,,,,"))
        self.assertFalse(value["position_valid"])
        self.assertIsNone(value["latitude_deg"])
        self.assertIsNone(value["longitude_deg"])
        self.assertIsNone(value["utc_time"])
        self.assertIsNone(value["altitude_ellipsoid_m"])
        self.assertEqual(value["satellites_used"], 0)
        partial = parse_sentence(sentence(GGA.replace("-12.50,M", ",M")))
        self.assertEqual(partial["altitude_msl_m"], 100.25)
        self.assertIsNone(partial["geoid_separation_m"])
        self.assertIsNone(partial["altitude_ellipsoid_m"])

    def test_checksum_required_for_supported_and_unsupported_sentences(self):
        unsupported = sentence("GPGSV,1,1,00")
        self.assertIsNone(parse_sentence(unsupported))
        for invalid in ("", GGA, "$" + GGA, sentence(GGA)[:-1], sentence(GGA)[:-2] + "XX",
                        sentence(GGA) + "garbage", sentence(GGA).replace("28", "29", 1),
                        unsupported.replace("00", "01", 1)):
            with self.subTest(invalid=invalid):
                self.assertRaises(ValueError, parse_sentence, invalid)
        self.assertEqual(parse_sentence(sentence(GGA)[:-2] + sentence(GGA)[-2:].lower())["fix_quality"], 4)

    def test_framing_rejects_binary_controls_concatenated_and_oversized(self):
        for invalid in (None, b"$GPGGA", " " + sentence(GGA), sentence(GGA) + "\n" + sentence(GGA),
                        sentence(GGA).replace("GNGGA", "GNG\x00GA"),
                        "$" + "x" * MAX_SENTENCE_LENGTH, sentence("GPGGA,é")):
            with self.subTest(invalid=repr(invalid)[:80]):
                self.assertRaises(ValueError, parse_sentence, invalid)
        self.assertIsNone(parse_sentence(sentence("PXYZ,ignored,1")))

    def test_geographic_boundaries_and_hemispheres(self):
        value = parse_sentence(sentence(GGA.replace("0955.1234567,N,08405.7654321,W", "9000.000,S,18000.000,E")))
        self.assertEqual((value["latitude_deg"], value["longitude_deg"]), (-90, 180))
        for bad in ("9060.0,N,08405.0,W", "9000.001,N,08405.0,W", "0955.0,N,18000.1,E",
                    "0955.0,Q,08405.0,W", "-0955.0,N,08405.0,W", "0960.0,N,08405.0,W",
                    "955.0,N,8405.0,W", ",,08405.0,W", "0955.0,N,,"):
            with self.subTest(bad=bad):
                self.assertRaises(ValueError, parse_sentence,
                                  sentence(GGA.replace("0955.1234567,N,08405.7654321,W", bad)))

    def test_numeric_fields_require_finite_and_nonnegative_when_applicable(self):
        fields = GGA.split(",")
        for index, invalids in ((6, ["", "-1", "10", "4.0"]), (7, ["-1", "1000", "2.5"]),
                                (8, ["nan", "inf", "-0.1", "1e2"]),
                                (9, ["NaN", "Inf", "9" * 400]), (13, ["-1", "NaN"])):
            for bad in invalids:
                with self.subTest(index=index, bad=bad[:20]):
                    changed = fields.copy(); changed[index] = bad
                    self.assertRaises(ValueError, parse_sentence, sentence(",".join(changed)))
        for index in (10, 12):
            changed = fields.copy(); changed[index] = "F"
            self.assertRaises(ValueError, parse_sentence, sentence(",".join(changed)))

    def test_time_boundaries_and_explicit_leap_second(self):
        for valid, expected in (("000000", 0), ("235959.5", 86399.5), ("235960", 86400)):
            result = parse_sentence(sentence(GGA.replace("040203.125", valid)))
            self.assertEqual(result["utc_seconds_of_day"], expected)
            self.assertEqual(result["utc_leap_second"], valid == "235960")
        for bad in ("240000", "126000", "120060", "04020", "040203.nan", "04:02:03", "040203.1234567890"):
            with self.subTest(bad=bad):
                self.assertRaises(ValueError, parse_sentence, sentence(GGA.replace("040203.125", bad)))

    def test_rmc_full_utc_date_and_optional_versions(self):
        value = parse_sentence(sentence(RMC))
        self.assertEqual(value["utc_date"], "2026-10-30")
        self.assertEqual(value["date_century_policy"], "1980-2079")
        self.assertEqual(value["mode"], "R")
        self.assertTrue(value["position_valid"])
        self.assertEqual(value["speed_knots"], 1.25)
        self.assertEqual(value["course_deg"], 123.4)
        self.assertIsNone(parse_sentence(sentence(RMC[:-2]))["mode"])
        self.assertEqual(parse_sentence(sentence(RMC + ",S"))["navigation_status"], "S")
        self.assertEqual(parse_sentence(sentence(RMC.replace("301026", "230394")))["utc_date"], "1994-03-23")

    def test_rmc_invalid_or_missing_position_never_promoted(self):
        value = parse_sentence(sentence("GNRMC,040203,V,,,,,,,301026,,,N"))
        self.assertFalse(value["position_valid"])
        self.assertEqual(value["utc_date"], "2026-10-30")
        self.assertIsNone(value["latitude_deg"])
        self.assertFalse(parse_sentence(sentence(RMC[:-1] + "N"))["position_valid"])
        self.assertRaises(ValueError, parse_sentence, sentence("GNRMC,040203,A,,,,,,,301026,,,A"))

    def test_rmc_rejects_invalid_date_status_course_speed_and_variation(self):
        fields = RMC.split(",")
        for index, bad in ((2, "Q"), (7, "-1"), (8, "360.1"), (9, "310226"), (9, "290223"),
                           (9, "30102026"), (10, "1"), (11, "E"), (12, "FIX")):
            with self.subTest(index=index, bad=bad):
                changed = fields.copy(); changed[index] = bad
                self.assertRaises(ValueError, parse_sentence, sentence(",".join(changed)))
        changed = fields.copy(); changed[10:12] = ["2.5", "W"]
        self.assertEqual(parse_sentence(sentence(",".join(changed)))["magnetic_variation_direction"], "W")

    def test_zda_full_date_preserves_utc_despite_zone(self):
        result = parse_sentence(sentence("GPZDA,040203.125,30,10,2026,-06,00"))
        self.assertEqual(result["utc_date"], "2026-10-30")
        self.assertEqual(result["utc_time"], "04:02:03.125")
        self.assertEqual(result["local_zone_offset_minutes"], -360)
        negative_zero = parse_sentence(sentence("GPZDA,000000,29,02,2024,-00,30"))
        self.assertEqual(negative_zero["local_zone_offset_minutes"], -30)
        self.assertIsNone(parse_sentence(sentence("GPZDA,000000,30,10,2026,,"))["local_zone_offset_minutes"])
        self.assertIsNone(parse_sentence(sentence("GPZDA,,,,,,"))["utc_date"])

    def test_zda_rejects_calendar_and_zone_errors(self):
        for bad in ("GPZDA,040203,29,02,2023,,", "GPZDA,040203,30,,2026,,",
                    "GPZDA,040203,30,10,26,,", "GPZDA,040203,30,10,2026,-06,60",
                    "GPZDA,040203,30,10,2026,,30", "GPZDA,040203,30,10,2026,99,00"):
            with self.subTest(bad=bad):
                self.assertRaises(ValueError, parse_sentence, sentence(bad))

    def test_gst_keeps_range_rms_separate_from_position_sigma(self):
        result = parse_sentence(sentence("GNGST,040203.125,1.25,0.02,0.01,-16.7566,0.02,0.01,0.03"))
        self.assertEqual(result["range_rms_m"], 1.25)
        self.assertEqual(result["sigma_latitude_m"], 0.02)
        self.assertEqual(result["sigma_longitude_m"], 0.01)
        self.assertEqual(result["sigma_altitude_m"], 0.03)
        self.assertEqual(result["ellipse_orientation_deg"], -16.7566)
        self.assertNotIn("utc_date", result)
        self.assertIsNone(parse_sentence(sentence("GPGST,040203,,,,,,,"))["sigma_altitude_m"])
        self.assertRaises(ValueError, parse_sentence, sentence("GNGST,040203,1.25,-0.02,0.01,0,0.02,0.01,0.03"))

    def test_supported_message_field_counts_reject_truncation_and_extra_fields(self):
        for body in (GGA.rsplit(",", 1)[0], GGA + ",", RMC.rsplit(",", 2)[0], RMC + ",S,extra",
                     "GPZDA,040203,30,10,2026", "GPGST,040203,1,2,3"):
            with self.subTest(body=body):
                self.assertRaises(ValueError, parse_sentence, sentence(body))


if __name__ == "__main__":
    unittest.main()
