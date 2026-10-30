"""Recorder behavior with an in-memory HTTP transport and synthetic positions.

No test contacts the GNSS, changes receiver configuration, or writes output/.
"""
import csv
from email.message import Message
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from urllib.request import BaseHandler
from urllib.response import addinfourl

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "gnss"))
import record_z1


class MemoryHTTP(BaseHandler):
    """Replace socket access, retaining urllib's redirect/error processing."""
    handler_order = 400

    def __init__(self, body=b'{"Device":"Z1"}', *, status=200, location=None):
        self.body = body
        self.status = status
        self.location = location
        self.requests = []

    def http_open(self, request):
        self.requests.append(request)
        headers = Message()
        headers["Content-Type"] = "application/json"
        if self.location:
            headers["Location"] = self.location
        response = addinfourl(io.BytesIO(self.body), headers, request.full_url, code=self.status)
        response.msg = "OK" if self.status == 200 else "Found"
        return response

    https_open = http_open


class Clock:
    def __init__(self):
        self.now = 1000.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeClient:
    base_url = "http://192.168.10.12:80/cgi-bin/"

    def __init__(self, entries, *, device="Z1"):
        self.entries = iter(entries)
        self.device = device
        self.calls = []
        self.index = 0

    def read(self, query):
        self.calls.append(query)
        if query == "device":
            return {"Device": self.device, "SWV": "fixture"}, {}
        if query == "status":
            return {"WorkMode": "Rover"}, {}
        if query != "position":
            raise AssertionError("Recorder attempted a configuration write")
        value = next(self.entries)
        if isinstance(value, BaseException):
            raise value
        self.index += 1
        return value, {
            "request_started_monotonic_ns": self.index * 1_000_000_000,
            "received_monotonic_ns": self.index * 1_000_000_000 + 50_000_000,
            "received_unix_ns": 1_700_000_000_000_000_000 + self.index * 1_000_000_000,
            "round_trip_ms": 50.0,
        }


def fix(seconds, *, kind="rtk_fixed", valid=True, usable=True):
    # Deliberately synthetic, not a surveyed control or a sample from the user.
    return {
        "gps_week": 2442, "gps_seconds": float(seconds),
        "utc_timestamp": "2026-10-30T04:00:00+00:00",
        "solution_status": "SOL_COMPUTED" if valid else "INSUFFICIENT_OBS",
        "position_type": "NARROW_INT" if kind == "rtk_fixed" else "NARROW_FLOAT",
        "fix_type": kind, "position_valid": valid,
        "latitude_deg": 1.0, "longitude_deg": 2.0,
        "height_reported_m": 100.0, "undulation_m": -10.0,
        "height_ellipsoid_m": 90.0, "datum": "WGS84",
        "sigma_latitude_m": 0.01, "sigma_longitude_m": 0.02,
        "sigma_height_m": 0.03, "correction_age_s": 1.0, "solution_age_s": 0.0,
        "satellites_tracked": 28, "satellites_used": 20,
        "epoch_coherent": True, "time_status": "FINESTEERING",
        "time_valid": True, "quality_usable": valid and usable,
        "height_reference_verified": False, "datum_reference_verified": False,
    }


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="atlas-z1-recorder-test-")
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "capture"

    def capture(self, entries, parsed=None, *, seconds=None, device="Z1"):
        client = FakeClient(entries, device=device)
        def parse(response):
            value = (parsed or {})[response["fixture"]]
            if isinstance(value, BaseException):
                raise value
            return value
        with patch.object(record_z1, "time", Clock()), \
                patch.object(record_z1, "parse_position", side_effect=parse), \
                patch("sys.stdout", new=io.StringIO()), patch("sys.stderr", new=io.StringIO()):
            code = record_z1.record(client, self.path, seconds=len(entries) if seconds is None else seconds,
                                    interval=1)
        metadata = json.loads((self.path / "session.json").read_text())
        csv_path = self.path / "fixes.csv"
        with csv_path.open() if csv_path.exists() else io.StringIO() as source:
            rows = list(csv.DictReader(source))
        raw_path = self.path / "raw.jsonl"
        raw = [json.loads(line) for line in raw_path.read_text().splitlines()] if raw_path.exists() else []
        return code, metadata, rows, raw, client

    def test_one_epoch_keeps_receiver_time_and_host_receipt_distinct(self):
        code, metadata, rows, raw, client = self.capture([{"fixture": "one"}], {"one": fix(100)})
        self.assertEqual(code, 0)
        self.assertEqual(metadata["state"], "recorded")
        self.assertEqual((metadata["unique_epochs"], metadata["fixed_epochs"]), (1, 1))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["gps_seconds"], "100.0")
        self.assertEqual(rows[0]["received_monotonic_ns"], "1050000000")
        self.assertEqual(raw[0]["response"], {"fixture": "one"})
        self.assertFalse(metadata["clock_alignment_verified"])
        self.assertFalse(metadata["lever_arm_verified"])
        self.assertFalse(metadata["georeferenced_lidar"])
        self.assertFalse(metadata["receiver_configuration_modified"])
        self.assertEqual(metadata["measurement_role"], "gnss_control_observation")
        self.assertEqual(client.calls, ["device", "status", "position"])

    def test_duplicates_and_older_epochs_stay_raw_and_quality_is_not_promoted(self):
        fixtures = {"fixed": fix(100), "duplicate": fix(100), "old": fix(99),
                    "float": fix(101, kind="rtk_float"), "invalid": fix(102, valid=False),
                    "uncertain_fixed": fix(103, usable=False)}
        code, metadata, rows, raw, _ = self.capture([{"fixture": key} for key in fixtures], fixtures)
        self.assertEqual(code, 0)
        self.assertEqual(metadata["unique_epochs"], 4)
        self.assertEqual(metadata["duplicate_epochs"], 1)
        self.assertEqual(metadata["out_of_order_epochs"], 1)
        self.assertEqual(metadata["fixed_epochs"], 1)
        self.assertEqual(metadata["reported_fixed_epochs"], 2)
        self.assertEqual([row["gps_seconds"] for row in rows], ["100.0", "101.0", "102.0", "103.0"])
        self.assertEqual(rows[1]["fix_type"], "rtk_float")
        self.assertEqual(rows[2]["position_valid"], "False")
        self.assertEqual(rows[3]["fix_type"], "rtk_fixed")
        self.assertEqual(rows[3]["quality_usable"], "False")
        self.assertEqual(len(raw), 6)
        self.assertEqual(raw[1]["excluded_from_csv"], "duplicate_epoch")
        self.assertEqual(raw[2]["excluded_from_csv"], "out_of_order_epoch")

    def test_bad_parse_retains_original_response_and_marks_partial_session(self):
        bad = {"fixture": "bad", "BESTPOSA": "original corrupt receiver data"}
        code, metadata, rows, raw, _ = self.capture(
            [{"fixture": "good"}, bad, OSError("fixture transport timeout")],
            {"good": fix(100), "bad": ValueError("fixture checksum invalid")})
        self.assertEqual(code, 2)
        self.assertEqual(metadata["state"], "partial")
        self.assertEqual(metadata["errors"], 2)
        self.assertEqual(len(rows), 1)
        self.assertEqual(len(raw), 3)
        self.assertEqual(raw[1]["response"], bad)
        self.assertIn("checksum", raw[1]["error"])
        self.assertNotIn("parsed", raw[1])
        self.assertIn("timeout", raw[2]["error"])

    def test_no_valid_epoch_fails_without_claiming_a_completed_capture(self):
        code, metadata, rows, raw, _ = self.capture([OSError("fixture timeout")])
        self.assertEqual(code, 2)
        self.assertEqual(metadata["state"], "failed")
        self.assertEqual(metadata["unique_epochs"], 0)
        self.assertEqual(rows, [])
        self.assertEqual(len(raw), 1)

    def test_device_mismatch_stops_before_position_collection(self):
        code, metadata, rows, raw, client = self.capture([], device="DifferentDevice", seconds=1)
        self.assertEqual(code, 2)
        self.assertEqual(metadata["state"], "failed")
        self.assertEqual(metadata["stop_reason"], "error")
        self.assertNotIn("position", client.calls)
        self.assertEqual(rows, [])
        self.assertEqual(raw, [])

    def test_existing_directory_is_preserved_and_never_opens_the_receiver(self):
        self.path.mkdir()
        original = b'{"existing":"must survive"}\n'
        (self.path / "session.json").write_bytes(original)
        (self.path / "notes.txt").write_text("Keep my notes")
        client = FakeClient([])
        with self.assertRaises(FileExistsError):
            record_z1.record(client, self.path, seconds=1)
        self.assertEqual(client.calls, [])
        self.assertEqual((self.path / "session.json").read_bytes(), original)
        self.assertEqual(set(p.name for p in self.path.iterdir()), {"session.json", "notes.txt"})

    def test_explicit_interrupt_retains_the_already_written_epoch(self):
        code, metadata, rows, raw, _ = self.capture(
            [{"fixture": "one"}, KeyboardInterrupt()], {"one": fix(100)}, seconds=0)
        self.assertEqual(code, 0)
        self.assertEqual(metadata["stop_reason"], "user_interrupt")
        self.assertEqual(metadata["state"], "recorded")
        self.assertEqual(len(rows), 1)
        self.assertEqual(len(raw), 1)


class ClientTests(unittest.TestCase):
    def client(self, body=b'{"Device":"Z1"}', **transport_options):
        client = record_z1.Z1Client(timeout=0.5)
        transport = MemoryHTTP(body, **transport_options)
        client.opener.add_handler(transport)
        return client, transport

    def test_only_observed_read_queries_are_sent_as_empty_posts(self):
        client, transport = self.client()
        for query in ("device", "status", "position"):
            response, receipt = client.read(query)
            self.assertEqual(response, {"Device": "Z1"})
            self.assertGreaterEqual(receipt["received_monotonic_ns"], receipt["request_started_monotonic_ns"])
        for query in ("config", "../config", "position.cgi", "", "http://example.com"):
            with self.subTest(query=query):
                self.assertRaises(ValueError, client.read, query)
        self.assertEqual(len(transport.requests), 3)
        for request, name in zip(transport.requests, ("device", "status", "position")):
            self.assertEqual(request.full_url, f"http://192.168.10.12:80/cgi-bin/{name}.cgi")
            self.assertEqual(request.get_method(), "POST")
            self.assertEqual(request.data, b"")
            self.assertEqual(request.timeout, 0.5)

    def test_proxy_environment_is_bypassed(self):
        with patch.dict("os.environ", {"http_proxy": "http://example.invalid:9999", "no_proxy": ""}):
            client, transport = self.client()
            client.read("position")
        self.assertFalse(transport.requests[0].has_proxy())
        self.assertEqual(transport.requests[0].host, "192.168.10.12:80")

    def test_redirect_is_rejected_without_contacting_the_target(self):
        client, transport = self.client(status=302, location="https://example.invalid/config")
        self.assertRaises(ValueError, client.read, "position")
        self.assertEqual(len(transport.requests), 1)

    def test_nonlocal_addresses_and_bad_limits_rejected(self):
        for host in ("8.8.8.8", "0.0.0.0", "224.0.0.1", "::1", "http://192.168.10.12", "example.com"):
            with self.subTest(host=host):
                self.assertRaises(ValueError, record_z1.Z1Client, host)
        for port in (0, -1, 65536):
            self.assertRaises(ValueError, record_z1.Z1Client, port=port)
        for timeout in (0, 0.49, 10.01, float("nan"), float("inf")):
            self.assertRaises(ValueError, record_z1.Z1Client, timeout=timeout)

    def test_size_json_shape_unicode_and_nonfinite_values_are_rejected(self):
        for payload in (b" " * (record_z1.MAX_RESPONSE_BYTES + 1), b"[]", b"null", b"invalid",
                        b'{"x":"\xff"}', b'{"x":NaN}', b'{"x":Infinity}', b'{"x":-Infinity}',
                        b'{"x":1e999}'):
            with self.subTest(payload=payload[:30]):
                client, _ = self.client(payload)
                self.assertRaises(ValueError, client.read, "position")


if __name__ == "__main__":
    unittest.main()
