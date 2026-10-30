"""Synthetic receiver messages only; no coordinates or identifiers from a user."""
import binascii
from copy import deepcopy
from datetime import datetime, timedelta, timezone
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "gnss"))
from z1_protocol import crc32, parse_position


def wire(name, fields, *, seconds="390643.125", week=2442, time_status="FINE", terminator="\\r\\n"):
    header = f"{name},COM2,100,65.0,{time_status},{week},{seconds},00000000,0,100"
    body = header + ";" + ",".join(fields)
    # Independent library implementation with the protocol's init/final XOR.
    checksum = binascii.crc32(body.encode("ascii"), 0xFFFFFFFF) ^ 0xFFFFFFFF
    return f"#{body}*{checksum:08x}" + terminator


def sample(*, position=None, time=None, position_header=None, time_header=None):
    p = ["SOL_COMPUTED", "NARROW_INT", "45.123456", "2.234567", "100.5", "25.25", "WGS84",
         "0.012", "0.010", "0.025", '"TEST"', "1.2", "0.0", "30", "26", "26", "23", "0", "00", "03", "fb"]
    # UTC epoch chosen independently; GPS reference time is UTC + 18 seconds.
    instant = datetime(2026, 10, 29, 12, 30, 25, 125000, tzinfo=timezone.utc)
    gps_seconds = (instant + timedelta(seconds=18) - datetime(1980, 1, 6, tzinfo=timezone.utc)).total_seconds()
    week, tow = divmod(gps_seconds, 604800)
    headers = {"week": int(week), "seconds": f"{tow:.3f}"}
    t = ["VALID", "0.000", "0.000000001", "-18.0", "2026", "10", "29", "12", "30", "25125", "VALID"]
    for index, value in (position or {}).items():
        p[index] = value
    for index, value in (time or {}).items():
        t[index] = value
    return {"BESTPOSA": wire("BESTPOSA", p, **{**headers, **(position_header or {})}),
            "TIMEA": wire("TIMEA", t, **{**headers, **(time_header or {})})}


class Z1ProtocolTests(unittest.TestCase):
    def test_crc_standard_polynomial_nonstandard_initialization(self):
        self.assertEqual(crc32(b"123456789"), 0x2DFD2D88)
        self.assertEqual(crc32(b""), 0)

    def test_fixed_solution_epochs_fields_and_explicit_reference_uncertainty(self):
        result = parse_position(sample())
        self.assertEqual(result["fix_type"], "rtk_fixed")
        self.assertTrue(result["quality_usable"])
        self.assertTrue(result["position_valid"])
        self.assertTrue(result["epoch_coherent"])
        self.assertTrue(result["utc_header_coherent"])
        self.assertEqual(result["utc_timestamp"], "2026-10-29T12:30:25.125Z")
        self.assertEqual(result["height_reported_m"], 100.5)
        self.assertEqual(result["undulation_m"], 25.25)
        self.assertEqual(result["height_ellipsoid_m"], 125.75)
        self.assertFalse(result["height_reference_verified"])
        self.assertFalse(result["datum_reference_verified"])
        self.assertEqual(result["datum"], "WGS84")
        self.assertEqual(result["satellites_tracked"], 30)
        self.assertEqual(result["satellites_used"], 26)
        self.assertEqual(result["correction_age_s"], 1.2)
        self.assertEqual(result["variance_enu_diagonal_m2"], [0.010**2, 0.012**2, 0.025**2])
        self.assertFalse(result["cross_covariances_available"])

    def test_quality_degradation_never_promotes_float_or_single_to_fixed(self):
        for position_type, expected in [("NARROW_FLOAT", "rtk_float"), ("SINGLE", "single"),
                                        ("PSRDIFF", "dgnss"), ("PPP", "ppp")]:
            with self.subTest(position_type=position_type):
                result = parse_position(sample(position={1: position_type}))
                self.assertEqual(result["fix_type"], expected)
                self.assertTrue(result["quality_usable"])
        invalid = parse_position(sample(position={0: "INSUFFICIENT_OBS"}))
        self.assertEqual(invalid["fix_type"], "invalid")
        self.assertFalse(invalid["quality_usable"])
        self.assertEqual(invalid["position_type"], "NARROW_INT")

    def test_unknown_or_fixed_base_types_are_not_usable_rover_positions(self):
        for value in ["VENDOR_UNKNOWN", "NONE", "FIXEDPOS", "FIXEDHEIGHT", "PROPAGATED"]:
            with self.subTest(position_type=value):
                result = parse_position(sample(position={1: value}))
                self.assertFalse(result["quality_usable"])
                self.assertFalse(result["position_valid"])
                self.assertEqual(result["position_type"], value)

    def test_missing_required_logs_and_wrong_response_type(self):
        for response in [None, [], {}, {"BESTPOSA": sample()["BESTPOSA"]}, {"TIMEA": sample()["TIMEA"]}]:
            with self.subTest(response_type=type(response)):
                with self.assertRaises(ValueError):
                    parse_position(response)

    def test_crc_is_required_for_position_and_time(self):
        for key in ["BESTPOSA", "TIMEA"]:
            for suffix in ["*00000000", "*bad", "", "*12345678garbage"]:
                response = sample()
                response[key] = response[key].split("*")[0] + suffix
                with self.subTest(key=key, suffix=suffix), self.assertRaises(ValueError):
                    parse_position(response)

    def test_header_and_payload_are_inside_crc(self):
        for before, after in [("COM2", "COM1"), ("45.123456", "45.123457"), ("NARROW_INT", "NARROW_FLOAT")]:
            response = sample()
            response["BESTPOSA"] = response["BESTPOSA"].replace(before, after)
            with self.subTest(change=before), self.assertRaises(ValueError):
                parse_position(response)

    def test_reject_nonfinite_values_even_with_valid_crc(self):
        for invalid in ["nan", "NaN", "inf", "-inf", "1e999"]:
            for index in [2, 3, 4, 5, 7, 8, 9, 11, 12]:
                with self.subTest(index=index, invalid=invalid), self.assertRaises(ValueError):
                    parse_position(sample(position={index: invalid}))
            for index in [1, 2, 3]:
                with self.subTest(time_index=index, invalid=invalid), self.assertRaises(ValueError):
                    parse_position(sample(time={index: invalid}))

    def test_position_sigma_age_satellite_ranges(self):
        for fields in [{2: "90.0001"}, {2: "-91"}, {3: "180.01"}, {3: "-180.001"},
                       {7: "-0.1"}, {9: "-1"}, {11: "-0.01"}, {12: "-1"},
                       {13: "256"}, {14: "31"}, {14: "2.5"}, {13: "-1"}]:
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                parse_position(sample(position=fields))

    def test_rtk_correction_age_gate_keeps_reported_fixed_type(self):
        self.assertTrue(parse_position(sample(position={11: "5.000"}))["quality_usable"])
        stale = parse_position(sample(position={11: "5.001"}))
        self.assertFalse(stale["quality_usable"])
        self.assertEqual(stale["fix_type"], "rtk_fixed")
        self.assertIn("rtk_corrections_older_than_5s", stale["quality_rejection_reasons"])

    def test_zero_sigma_or_no_satellites_are_not_measurement_confidence(self):
        for fields in [{7: "0"}, {8: "0"}, {9: "0"}, {14: "0"}]:
            with self.subTest(fields=fields):
                self.assertFalse(parse_position(sample(position=fields))["quality_usable"])

    def test_unknown_or_impossible_utc_date_never_uses_host_clock(self):
        for fields in [{5: "0", 6: "0"}, {4: "0"}, {5: "2", 6: "31"}]:
            with self.subTest(fields=fields):
                result = parse_position(sample(time=fields))
                self.assertIsNone(result["utc_timestamp"])
                self.assertFalse(result["time_valid"])
                self.assertFalse(result["quality_usable"])

    def test_clock_utc_and_header_status_are_gated_independently(self):
        for fields in [{0: "INVALID"}, {10: "INVALID"}, {10: "WARNING"}]:
            with self.subTest(fields=fields):
                result = parse_position(sample(time=fields))
                self.assertFalse(result["quality_usable"])
                self.assertIsNone(result["utc_timestamp"])
        result = parse_position(sample(position_header={"time_status": "COARSE"}))
        self.assertFalse(result["quality_usable"])
        self.assertTrue(result["time_valid"])

    def test_separate_position_and_time_epochs_are_not_paired(self):
        for fields in [{"seconds": "390644.125"}, {"week": 2441}]:
            with self.subTest(header=fields):
                result = parse_position(sample(time_header=fields))
                self.assertFalse(result["epoch_coherent"])
                self.assertFalse(result["quality_usable"])
                self.assertEqual(result["fix_type"], "rtk_fixed")

    def test_date_conflicting_with_gps_header_is_not_usable(self):
        result = parse_position(sample(time={6: "30"}))
        self.assertTrue(result["time_valid"])
        self.assertTrue(result["epoch_coherent"])
        self.assertFalse(result["utc_header_coherent"])
        self.assertFalse(result["quality_usable"])

    def test_submillisecond_clock_offset_allows_calendar_quantization(self):
        self.assertTrue(parse_position(sample(time={1: "-0.000433"}))["quality_usable"])

    def test_gps_week_and_seconds_must_be_bounded(self):
        for fields in [{"week": -1}, {"week": "2442.0"}, {"seconds": "604800"},
                       {"seconds": "-0.1"}, {"seconds": "nan"}]:
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                parse_position(sample(position_header=fields))

    def test_leap_second_is_flagged_without_mapping_to_wrong_unix_epoch(self):
        result = parse_position(sample(time={7: "23", 8: "59", 9: "60125"}))
        self.assertTrue(result["utc_leap_second"])
        self.assertIsNone(result["utc_timestamp"])
        self.assertFalse(result["quality_usable"])

    def test_actual_and_literal_line_endings_are_equivalent(self):
        base = sample()
        for suffix in ["\r\n", "\n", ""]:
            altered = {key: value.removesuffix("\\r\\n") + suffix for key, value in base.items()}
            self.assertEqual(parse_position(base), parse_position(altered))

    def test_unexpected_fields_framing_and_message_names_are_rejected(self):
        for sentence in ["garbage", "#BESTPOSA,broken*12345678", sample()["BESTPOSA"] + "extra",
                         sample()["BESTPOSA"].replace("SOL_COMPUTED", "SOL_\nCOMPUTED")]:
            response = sample()
            response["BESTPOSA"] = sentence
            with self.subTest(sentence=sentence[:20]), self.assertRaises(ValueError):
                parse_position(response)

    def test_does_not_mutate_input_or_invent_lidar_alignment(self):
        response = sample()
        original = deepcopy(response)
        result = parse_position(response)
        self.assertEqual(response, original)
        self.assertNotIn("lidar_timestamp", result)
        self.assertNotIn("georeferenced", result)


if __name__ == "__main__":
    unittest.main()
