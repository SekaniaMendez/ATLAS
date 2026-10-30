"""Validate the Z1 WebUI's BESTPOSA/TIMEA snapshot without contacting hardware.

The observed Z1 firmware uses the NovAtel-compatible ASCII layout and CRC:
https://docs.novatel.com/OEM7/Content/Messages/ASCII.htm
https://docs.novatel.com/OEM7/Content/Messages/32_Bit_CRC.htm
https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm
https://docs.novatel.com/OEM7/Content/Logs/TIME.htm

Compatibility of fields does not establish the physical antenna reference, the
base station's datum, a geoid model, or synchronization with the LiDAR clock.
"""
from __future__ import annotations

import csv
from datetime import datetime, timedelta, timezone
from decimal import Decimal, InvalidOperation
import math
import re


_CRC = re.compile(r"[0-9A-Fa-f]{8}")
_UINT = re.compile(r"[0-9]+")
_TOKEN = re.compile(r"[A-Z][A-Z0-9_]*")
_GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
_FINE_TIME = {"FINE", "FINESTEERING", "FINEBACKUPSTEERING"}
_FIX_TYPES = {
    "NARROW_INT": "rtk_fixed", "WIDE_INT": "rtk_fixed", "L1_INT": "rtk_fixed",
    "NARROW_FLOAT": "rtk_float", "L1_FLOAT": "rtk_float", "IONOFREE_FLOAT": "rtk_float",
    "PSRDIFF": "dgnss", "WAAS": "sbas", "SINGLE": "single",
    "PPP": "ppp", "PPP_BASIC": "ppp", "PPP_CONVERGING": "ppp_converging",
    "NONE": "invalid", "FIXEDPOS": "fixed_position", "FIXEDHEIGHT": "fixed_height",
    "PROPAGATED": "propagated",
}


def crc32(data: bytes) -> int:
    """Reflected CRC-32, initial value zero and no final XOR (not zlib defaults)."""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xEDB88320 if value & 1 else 0)
    return value


def _uint(value, field, maximum=2**32 - 1):
    if not _UINT.fullmatch(value):
        raise ValueError(f"Invalid {field}: expected unsigned integer")
    number = int(value)
    if number > maximum:
        raise ValueError(f"Invalid {field}: outside allowed range")
    return number


def _number(value, field, minimum=None, maximum=None):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"Invalid {field}: expected finite number") from error
    if (not math.isfinite(number) or (minimum is not None and number < minimum)
            or (maximum is not None and number > maximum)):
        raise ValueError(f"Invalid {field}: outside allowed range or non-finite")
    return number


def _token(value, field):
    if not _TOKEN.fullmatch(value):
        raise ValueError(f"Invalid {field}")
    return value


def _log(value, name, count):
    if not isinstance(value, str) or not 20 <= len(value) <= 4096:
        raise ValueError(f"Missing or invalid {name} sentence")
    # The Z1 JSON endpoint returns a literal backslash-r/backslash-n suffix on
    # some firmware. Strip only that suffix, never unescape arbitrary content.
    if value.endswith("\\r\\n"):
        value = value[:-4]
    else:
        value = value.removesuffix("\r\n").removesuffix("\n")
    if (not value.startswith("#") or value.count("*") != 1
            or any(ord(char) < 32 or ord(char) > 126 for char in value)):
        raise ValueError(f"Invalid {name} framing")
    body, checksum = value[1:].split("*")
    if not _CRC.fullmatch(checksum) or crc32(body.encode("ascii")) != int(checksum, 16):
        raise ValueError(f"Invalid {name} CRC32")
    if body.count(";") != 1:
        raise ValueError(f"Invalid {name} header separator")
    header, payload = body.split(";")
    header = header.split(",")
    try:
        fields = next(csv.reader([payload], strict=True))
    except csv.Error as error:
        raise ValueError(f"Invalid {name} CSV fields") from error
    if len(header) != 10 or header[0] != name or len(fields) != count:
        raise ValueError(f"Invalid {name} field count or message name")
    week = _uint(header[5], f"{name} GPS week", 100000)
    seconds = _number(header[6], f"{name} GPS seconds", 0)
    if seconds >= 604800:
        raise ValueError(f"Invalid {name} GPS seconds: outside week")
    try:
        seconds_exact = Decimal(header[6])
    except InvalidOperation as error:
        raise ValueError(f"Invalid {name} GPS seconds") from error
    if not seconds_exact.is_finite():
        raise ValueError(f"Invalid {name} GPS seconds")
    return {"gps_week": week, "gps_seconds": seconds,
            "gps_seconds_exact": seconds_exact,
            "time_status": _token(header[4], f"{name} time status")}, fields


def _time(header, fields):
    clock_status = _token(fields[0], "clock model status")
    offset = _number(fields[1], "receiver clock offset")
    std = _number(fields[2], "receiver clock standard deviation", 0)
    utc_offset = _number(fields[3], "GPS to UTC offset")
    year, month, day, hour, minute, milliseconds = (
        _uint(fields[index], f"TIMEA {name}", limit)
        for index, name, limit in [(4, "year", 9999), (5, "month", 12), (6, "day", 31),
                                   (7, "hour", 23), (8, "minute", 59), (9, "milliseconds", 60999)])
    utc_status = _token(fields[10], "UTC status")
    instant = None
    leap = milliseconds >= 60000
    try:
        base = datetime(year, month, day, hour, minute, tzinfo=timezone.utc)
        if not leap:
            instant = base + timedelta(milliseconds=milliseconds)
    except ValueError:
        pass  # Unknown UTC date uses zero month/day on the wire.
    valid = clock_status == "VALID" and utc_status == "VALID" and instant is not None
    consistent = False
    if valid:
        try:
            implied = _GPS_EPOCH + timedelta(weeks=header["gps_week"],
                seconds=header["gps_seconds"] - offset + utc_offset)
            # The header and UTC calendar fields have millisecond granularity.
            consistent = abs((instant - implied).total_seconds()) <= 0.002
        except (OverflowError, ValueError):
            pass
    return {"clock_status": clock_status, "utc_status": utc_status,
            "receiver_clock_offset_s": offset, "receiver_clock_sigma_s": std,
            "gps_to_utc_offset_s": utc_offset, "time_valid": valid,
            "utc_timestamp": instant.isoformat(timespec="milliseconds").replace("+00:00", "Z") if valid else None,
            "utc_leap_second": leap, "utc_header_coherent": consistent}


def parse_position(response: dict) -> dict:
    """Return checked receiver fields; malformed/CRC-invalid snapshots raise ValueError.

    ``fix_type`` is the receiver-reported solution, not a quality guarantee.
    ``quality_usable`` means only internally coherent computed position/time;
    RTK correction age above five seconds is rejected. Caller must additionally
    gate RTK FIX, sigma, freshness and calibration.
    Unknown dates/clock validity and mismatched epochs remain observable but are
    never marked usable. No timestamps are inferred from the Mac's wall clock.
    """
    if not isinstance(response, dict):
        raise ValueError("Position response must be a JSON object")
    position_header, fields = _log(response.get("BESTPOSA"), "BESTPOSA", 21)
    time_header, time_fields = _log(response.get("TIMEA"), "TIMEA", 11)
    status = _token(fields[0], "solution status")
    position_type = _token(fields[1], "position type")
    reported_type = _FIX_TYPES.get(position_type, "unknown")
    height = _number(fields[4], "reported height")
    undulation = _number(fields[5], "undulation")
    ellipsoid = height + undulation
    if not math.isfinite(ellipsoid):
        raise ValueError("Invalid derived height: non-finite")
    sigma = [_number(fields[index], f"{axis} sigma", 0, 1e9)
             for index, axis in [(7, "latitude"), (8, "longitude"), (9, "height")]]
    tracked = _uint(fields[13], "satellites tracked", 255)
    used = _uint(fields[14], "satellites used", 255)
    if used > tracked:
        raise ValueError("Satellites used exceed satellites tracked")
    # A fixed base position, propagated estimate or constrained height is not a
    # measured rover trajectory even when SOL_COMPUTED appears alongside it.
    position_valid = status == "SOL_COMPUTED" and reported_type not in {
        "unknown", "invalid", "fixed_position", "fixed_height", "propagated"}
    epoch_coherent = (position_header["gps_week"] == time_header["gps_week"]
                      and position_header["gps_seconds_exact"] == time_header["gps_seconds_exact"])
    result = {"gps_week": position_header["gps_week"], "gps_seconds": position_header["gps_seconds"],
        "time_status": position_header["time_status"], "time_log_status": time_header["time_status"],
        "solution_status": status, "position_type": position_type,
        "fix_type": reported_type if status == "SOL_COMPUTED" else "invalid",
        "position_valid": position_valid, "epoch_coherent": epoch_coherent,
        "latitude_deg": _number(fields[2], "latitude", -90, 90),
        "longitude_deg": _number(fields[3], "longitude", -180, 180),
        "height_reported_m": height, "undulation_m": undulation,
        "height_ellipsoid_m": ellipsoid, "datum": _token(fields[6], "datum"),
        "height_reference_verified": False, "datum_reference_verified": False,
        "height_conversion": "BESTPOS convention: reported height + undulation; Z1 physical/geoid reference unverified",
        "sigma_latitude_m": sigma[0], "sigma_longitude_m": sigma[1], "sigma_height_m": sigma[2],
        "variance_enu_diagonal_m2": [sigma[1]**2, sigma[0]**2, sigma[2]**2],
        "cross_covariances_available": False,
        "base_station_id": fields[10],
        "correction_age_s": _number(fields[11], "correction age", 0),
        "solution_age_s": _number(fields[12], "solution age", 0),
        "satellites_tracked": tracked, "satellites_used": used,
        **_time(time_header, time_fields)}
    reasons = []
    if not position_valid:
        reasons.append("position_not_computed_or_supported_rover_type")
    if not epoch_coherent:
        reasons.append("position_time_epoch_mismatch")
    if not result["time_valid"]:
        reasons.append("utc_clock_or_date_invalid")
    if not result["utc_header_coherent"]:
        reasons.append("utc_gps_header_mismatch")
    if position_header["time_status"] not in _FINE_TIME or time_header["time_status"] not in _FINE_TIME:
        reasons.append("gps_time_not_fine")
    if used == 0:
        reasons.append("no_satellites_used")
    if any(value == 0 for value in sigma):
        reasons.append("uncertainty_unavailable")
    if result["fix_type"] in {"rtk_fixed", "rtk_float"} and result["correction_age_s"] > 5:
        reasons.append("rtk_corrections_older_than_5s")
    result["quality_usable"] = not reasons
    result["quality_rejection_reasons"] = reasons
    return result
