"""Decode individual GNSS NMEA sentences without assigning a reception timestamp.

The receiver's UTC fields are kept separate from any host receipt time. GGA and
GST do not contain a date; callers must not infer one from the computer clock.
RMC has a two-digit year, expanded with a fixed 1980--2079 window, not a moving
window. ZDA carries the full year. Neither establishes GNSS/LiDAR clock alignment.

Null numeric fields are None, not zero. Heights follow the GGA convention:
ellipsoidal height = MSL height + geoid separation, only when both are present.
The receiver's actual horizontal/vertical datum still needs to be configured and
recorded separately. FIX/FLOAT are the reported NMEA quality, not an independent
accuracy check. GST range RMS is not a substitute for carrier-phase position SD.

Field references (receiver-independent NMEA layouts):
https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm
https://docs.novatel.com/OEM7/Content/Logs/GPZDA.htm
https://docs.novatel.com/OEM7/Content/Logs/GPGST.htm
"""
from __future__ import annotations

from datetime import date
import math
import re


# Precision-extended GNSS sentences may exceed the traditional 82-byte NMEA
# limit. Bound memory without rejecting those documented receiver extensions.
MAX_SENTENCE_LENGTH = 1024
RMC_YEAR_PIVOT = 80
_DECIMAL = re.compile(r"-?(?:\d+(?:\.\d*)?|\.\d+)\Z", re.ASCII)
_INTEGER = re.compile(r"\d+\Z", re.ASCII)
_TIME = re.compile(r"(\d{2})(\d{2})(\d{2})(\.\d{1,9})?\Z", re.ASCII)
_FIX_TYPES = {
    0: "invalid", 1: "standalone", 2: "differential", 3: "pps",
    4: "rtk_fixed", 5: "rtk_float", 6: "dead_reckoning",
    7: "manual", 8: "simulation", 9: "vendor_specific",
}


def _number(value, field, *, minimum=None, maximum=None):
    if value == "":
        return None
    if not _DECIMAL.fullmatch(value):
        raise ValueError(f"Invalid NMEA {field}: expected a decimal number")
    result = float(value)
    if (not math.isfinite(result) or (minimum is not None and result < minimum)
            or (maximum is not None and result > maximum)):
        raise ValueError(f"Invalid NMEA {field}: value outside its allowed range")
    return result


def _integer(value, field, *, minimum=0, maximum=None, required=False):
    if value == "" and not required:
        return None
    if not _INTEGER.fullmatch(value):
        raise ValueError(f"Invalid NMEA {field}: expected an integer")
    result = int(value)
    if result < minimum or (maximum is not None and result > maximum):
        raise ValueError(f"Invalid NMEA {field}: integer outside its allowed range")
    return result


def _utc_time(value):
    if not value:
        return {"utc_time": None, "utc_seconds_of_day": None, "utc_leap_second": False}
    match = _TIME.fullmatch(value)
    if not match:
        raise ValueError("Invalid NMEA UTC time: expected hhmmss[.fraction]")
    hour, minute, second = (int(match[index]) for index in (1, 2, 3))
    leap = second == 60 and hour == 23 and minute == 59
    if hour > 23 or minute > 59 or (second > 59 and not leap):
        raise ValueError("Invalid NMEA UTC time: components outside their allowed range")
    fraction = match[4] or ""
    return {"utc_time": f"{match[1]}:{match[2]}:{match[3]}{fraction}",
            "utc_seconds_of_day": hour * 3600 + minute * 60 + second + float("0" + fraction),
            "utc_leap_second": leap}


def _coordinate(value, hemisphere, *, latitude):
    name, digits, bound, hemispheres = (("latitude", 2, 90, "NS") if latitude
                                      else ("longitude", 3, 180, "EW"))
    if not value and not hemisphere:
        return None
    if hemisphere not in tuple(hemispheres) or not re.fullmatch(
            rf"[0-9]{{{digits + 2}}}(?:\.[0-9]+)?", value):
        raise ValueError(f"Invalid NMEA {name}: coordinates need degrees/minutes and hemisphere")
    degrees, minutes = int(value[:digits]), float(value[digits:])
    if not math.isfinite(minutes) or minutes >= 60 or degrees > bound or (degrees == bound and minutes != 0):
        raise ValueError(f"Invalid NMEA {name}: geographic limit exceeded")
    result = degrees + minutes / 60
    return -result if hemisphere in "SW" else result


def _position(latitude, north_south, longitude, east_west):
    lat = _coordinate(latitude, north_south, latitude=True)
    lon = _coordinate(longitude, east_west, latitude=False)
    if (lat is None) != (lon is None):
        raise ValueError("Invalid NMEA position: latitude and longitude must be supplied together")
    return {"latitude_deg": lat, "longitude_deg": lon}


def _height(value, unit, field):
    result = _number(value, field)
    if unit not in ("", "M") or (result is not None and unit != "M"):
        raise ValueError(f"Invalid NMEA {field}: expected metres (M)")
    return result


def _gga(fields):
    if len(fields) != 15:
        raise ValueError("Invalid GGA field count")
    result = _utc_time(fields[1])
    result.update(_position(*fields[2:6]))
    quality = _integer(fields[6], "fix quality", maximum=9, required=True)
    if quality and result["latitude_deg"] is None:
        raise ValueError("GGA reports a position fix without coordinates")
    msl = _height(fields[9], fields[10], "altitude above MSL")
    geoid = _height(fields[11], fields[12], "geoid separation")
    ellipsoid = msl + geoid if msl is not None and geoid is not None else None
    if ellipsoid is not None and not math.isfinite(ellipsoid):
        raise ValueError("Invalid GGA ellipsoidal height: finite sum required")
    station_id = fields[14] or None
    if station_id is not None and not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", station_id):
        raise ValueError("Invalid GGA differential station identifier")
    result.update(fix_quality=quality, fix_type=_FIX_TYPES[quality],
                  position_valid=quality != 0 and result["latitude_deg"] is not None,
                  satellites_used=_integer(fields[7], "satellite count", maximum=999),
                  hdop=_number(fields[8], "HDOP", minimum=0),
                  altitude_msl_m=msl, geoid_separation_m=geoid,
                  altitude_ellipsoid_m=ellipsoid,
                  correction_age_s=_number(fields[13], "correction age", minimum=0),
                  station_id=station_id)
    return result


def _rmc_date(value):
    if not value:
        return None
    if not re.fullmatch(r"[0-9]{6}", value):
        raise ValueError("Invalid RMC UTC date: expected ddmmyy")
    day, month, year = (int(value[index:index + 2]) for index in (0, 2, 4))
    year += 1900 if year >= RMC_YEAR_PIVOT else 2000
    try:
        return date(year, month, day).isoformat()
    except ValueError as error:
        raise ValueError("Invalid RMC UTC calendar date") from error


def _rmc(fields):
    # Legacy RMC; NMEA 2.3 adds mode; NMEA 4.1 adds navigation status.
    if not 12 <= len(fields) <= 14:
        raise ValueError("Invalid RMC field count")
    if fields[2] not in ("A", "V"):
        raise ValueError("Invalid RMC status: expected A or V")
    result = _utc_time(fields[1])
    result.update(_position(*fields[3:7]))
    if fields[2] == "A" and result["latitude_deg"] is None:
        raise ValueError("RMC reports a valid position without coordinates")
    variation = _number(fields[10], "magnetic variation", minimum=0, maximum=180)
    direction = fields[11] or None
    if ((variation is None) != (direction is None)
            or (direction is not None and direction not in ("E", "W"))):
        raise ValueError("Invalid RMC magnetic variation direction")
    mode = fields[12] or None if len(fields) >= 13 else None
    nav_status = fields[13] or None if len(fields) == 14 else None
    for value, name in ((mode, "mode"), (nav_status, "navigation status")):
        if value is not None and not re.fullmatch(r"[A-Z]", value):
            raise ValueError(f"Invalid RMC {name}")
    result.update(status=fields[2], mode=mode, navigation_status=nav_status,
                  position_valid=fields[2] == "A" and mode != "N",
                  utc_date=_rmc_date(fields[9]), utc_date_raw=fields[9] or None,
                  date_century_policy="1980-2079",
                  speed_knots=_number(fields[7], "speed", minimum=0),
                  course_deg=_number(fields[8], "course", minimum=0, maximum=360),
                  magnetic_variation_deg=variation, magnetic_variation_direction=direction)
    return result


def _zda(fields):
    if len(fields) != 7:
        raise ValueError("Invalid ZDA field count")
    result = _utc_time(fields[1])
    date_fields = fields[2:5]
    if not any(date_fields):
        utc_date = None
    else:
        if not (re.fullmatch(r"[0-9]{2}", fields[2]) and re.fullmatch(r"[0-9]{2}", fields[3])
                and re.fullmatch(r"[0-9]{4}", fields[4])):
            raise ValueError("Invalid ZDA UTC date: expected dd,mm,yyyy")
        try:
            utc_date = date(int(fields[4]), int(fields[3]), int(fields[2])).isoformat()
        except ValueError as error:
            raise ValueError("Invalid ZDA UTC calendar date") from error
    zone_hour, zone_minute = fields[5:7]
    offset = None
    if zone_hour or zone_minute:
        if not re.fullmatch(r"[+-]?[0-9]{2}", zone_hour):
            raise ValueError("Invalid ZDA local zone hour")
        hour = int(zone_hour)
        minute = _integer(zone_minute, "local zone minute", maximum=59, required=True)
        if abs(hour) > 13:
            raise ValueError("Invalid ZDA local zone hour: exceeds NMEA range")
        offset = (abs(hour) * 60 + minute) * (-1 if zone_hour.startswith("-") else 1)
    # The input time is already UTC; a zone descriptor never shifts it.
    result.update(utc_date=utc_date, local_zone_offset_minutes=offset)
    return result


def _gst(fields):
    if len(fields) != 9:
        raise ValueError("Invalid GST field count")
    result = _utc_time(fields[1])
    result.update(range_rms_m=_number(fields[2], "range RMS", minimum=0),
                  semi_major_sigma_m=_number(fields[3], "semi-major sigma", minimum=0),
                  semi_minor_sigma_m=_number(fields[4], "semi-minor sigma", minimum=0),
                  ellipse_orientation_deg=_number(fields[5], "ellipse orientation", minimum=-360, maximum=360),
                  sigma_latitude_m=_number(fields[6], "latitude sigma", minimum=0),
                  sigma_longitude_m=_number(fields[7], "longitude sigma", minimum=0),
                  sigma_altitude_m=_number(fields[8], "altitude sigma", minimum=0))
    return result


_PARSERS = {"GGA": _gga, "RMC": _rmc, "ZDA": _zda, "GST": _gst}


def parse_sentence(line: str) -> dict | None:
    """Parse one checksummed $ sentence; unknown sentence types return None.

    Raise ValueError for malformed framing, checksum or supported message data.
    CR/LF terminators are optional. Transport assembly and receipt clocks belong
    to the caller. Unsupported messages are checksum-checked before being skipped.
    """
    if not isinstance(line, str):
        raise ValueError("NMEA input must be a text sentence")
    if len(line) > MAX_SENTENCE_LENGTH:
        raise ValueError("NMEA sentence exceeds the supported length limit")
    sentence = line.rstrip("\r\n")
    if (not sentence.startswith("$") or sentence.count("*") != 1
            or any(not 32 <= ord(character) <= 126 for character in sentence)):
        raise ValueError("Invalid NMEA sentence framing")
    body, checksum = sentence[1:].split("*")
    if not re.fullmatch(r"[0-9a-fA-F]{2}", checksum):
        raise ValueError("NMEA requires a two-digit hexadecimal checksum")
    calculated = 0
    for character in body:
        calculated ^= ord(character)
    if calculated != int(checksum, 16):
        raise ValueError("NMEA checksum mismatch")
    fields = body.split(",")
    identifier = fields[0]
    if not re.fullmatch(r"[A-Z][A-Z0-9]{2,15}", identifier):
        raise ValueError("Invalid NMEA sentence identifier")
    if len(identifier) != 5 or identifier[2:] not in _PARSERS:
        return None
    if not re.fullmatch(r"[A-Z]{2}", identifier[:2]):
        raise ValueError("Invalid NMEA talker identifier")
    message_type = identifier[2:]
    result = _PARSERS[message_type](fields)
    return {"message_type": message_type, "talker": identifier[:2],
            "raw_sentence": sentence, **result}
