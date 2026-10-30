import importlib.util
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

MODULE = Path(__file__).resolve().parents[2] / "tools/calibration/estimate_time_offset.py"
spec = importlib.util.spec_from_file_location("estimate_time_offset", MODULE)
sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync)


def samples(kind="varied", mismatch=False):
    phone_t = np.arange(0, 24, 1 / 60)
    lidar_t = np.arange(-1.27, 26, 1 / 200)

    def omega(t):
        if kind == "static":
            return np.zeros_like(t)
        if kind == "periodic":
            return 0.8 + 0.5 * np.sin(t * 2 * np.pi)
        # Distinct nonperiodic bursts, including the reserved final segment.
        result = np.full_like(t, 0.12)
        for center, width, height in [(1.3, .2, 1.1), (3.9, .45, .7), (6.7, .3, 1.4),
                                       (8.4, .17, .9), (11.6, .5, 1.2), (13.8, .25, .8),
                                       (16.5, .3, 1.3), (19.2, .2, .8), (22.5, .45, 1.1)]:
            result += height * np.exp(-0.5 * ((t - center) / width) ** 2)
        return result

    # Integrate at high rate to avoid making the estimator's discretization its oracle.
    integration_t = np.arange(-0.01, 24.01, .0005)
    integrated = np.cumsum(omega(integration_t)) * .0005
    theta = np.interp(phone_t, integration_t, integrated)
    r = np.zeros((len(theta), 3, 3))
    r[:, 0, 0] = r[:, 1, 1] = np.cos(theta)
    r[:, 0, 1] = -np.sin(theta)
    r[:, 1, 0] = np.sin(theta)
    r[:, 2, 2] = 1
    gyro_t = lidar_t + np.where(lidar_t > 15.6, .25, 0) if mismatch else lidar_t
    speed = omega(gyro_t)
    gyro = np.column_stack((speed * .36, speed * .48, speed * .8))
    gyro += np.random.default_rng(421).normal(0, .002, gyro.shape)
    anchor = 8_000_000_000_000_000_000
    offset = -3_000_000_000_123_456_789
    pn = np.asarray([anchor + round(t * 1e9) for t in phone_t], dtype=np.uint64)
    ln = np.asarray([anchor + offset + round(t * 1e9) for t in lidar_t], dtype=np.uint64)
    return pn, r, np.ones(len(pn), dtype=bool), ln, gyro, offset


class TimeOffsetTests(unittest.TestCase):
    def estimate(self, kind="varied", mismatch=False):
        *args, offset = samples(kind, mismatch)
        return sync.estimate_offset(*args, search_seconds=3, step_seconds=.004), offset

    def test_known_offset_large_epochs_and_rotated_gyro(self):
        report, true_offset = self.estimate()
        self.assertEqual(report["status"], "candidate_requires_review", report)
        self.assertLess(abs(report["phone_to_lidar_offset_ns"] - true_offset), 5_000_000)
        self.assertFalse(report["clock_alignment_verified"])
        self.assertFalse(report["automatic_activation_allowed"])
        self.assertGreaterEqual(report["empirical_uncertainty_ns"], 8_000_000)

    def test_static_rejected(self):
        report, _ = self.estimate("static")
        self.assertEqual(report["status"], "rejected")
        self.assertTrue(any("excitation" in reason for reason in report["reasons"]))
        self.assertNotIn("phone_to_lidar_offset_ns", report)

    def test_repeated_motion_rejected(self):
        report, _ = self.estimate("periodic")
        self.assertEqual(report["status"], "rejected")
        self.assertTrue(any("ambiguous" in reason for reason in report["reasons"]), report)

    def test_reserved_segment_rejects_time_change(self):
        report, _ = self.estimate(mismatch=True)
        self.assertEqual(report["status"], "rejected")
        self.assertTrue(any("reserved" in reason for reason in report["reasons"]), report)

    def test_tracking_loss_and_clock_reset(self):
        pn, r, normal, ln, gyro, _ = samples()
        normal[:] = False
        report = sync.estimate_offset(pn, r, normal, ln, gyro, search_seconds=3)
        self.assertEqual(report["status"], "rejected")
        pn[100] = pn[99]
        with self.assertRaisesRegex(ValueError, "increase strictly"):
            sync.estimate_offset(pn, r, normal, ln, gyro)

    def test_pose_completion_and_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            row = dict(schema_version=1, sequence=0, timestamp_ns=123, tracking_normal=True,
                       world_from_camera_row_major=np.eye(4).ravel().tolist())
            (root / "poses.jsonl").write_text(json.dumps(row) + "\n")
            marker = dict(complete=True, pose_log_complete=True, pose_samples=1, pose_samples_dropped=0)
            (root / "complete.json").write_text(json.dumps(marker))
            stamps, rotations, normal = sync.load_poses(root)
            self.assertEqual(stamps.tolist(), [123])
            self.assertEqual(rotations.shape, (1, 3, 3))
            self.assertTrue(normal[0])
            marker["pose_samples"] = 2
            (root / "complete.json").write_text(json.dumps(marker))
            with self.assertRaisesRegex(ValueError, "counts"):
                sync.load_poses(root)

    def test_gyro_clock_mixture_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gyro.csv"
            path.write_text("timestamp_ns,clock_source,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s\n"
                            "123,0,0,0,0\n124,1,0,0,0\n")
            with self.assertRaisesRegex(ValueError, "exactly one"):
                sync.load_gyro(path)

    def test_cli_complete_camera_log_and_exporter_csv(self):
        pn, rotations, normal, ln, gyro, true_offset = samples()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with (root / "poses.jsonl").open("w") as stream:
                for i, stamp in enumerate(pn):
                    transform = np.eye(4)
                    transform[:3, :3] = rotations[i]
                    row = dict(schema_version=1, sequence=i, timestamp_ns=int(stamp),
                               tracking_normal=bool(normal[i]),
                               world_from_camera_row_major=transform.ravel().tolist())
                    stream.write(json.dumps(row) + "\n")
            (root / "complete.json").write_text(json.dumps(dict(
                complete=True, pose_log_complete=True, pose_samples=len(pn), pose_samples_dropped=0)))
            gyro_path = root / "gyro.csv"
            with gyro_path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["timestamp_ns", "clock_source", "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s"])
                writer.writerows([int(stamp), 0, *vector] for stamp, vector in zip(ln, gyro))
            report_path = root / "time-offset.json"
            command = [sys.executable, str(MODULE), "--poses", str(root), "--gyro", str(gyro_path),
                       "--output", str(report_path), "--search-seconds", "3", "--step-ms", "4"]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(report_path.read_text())
            self.assertLess(abs(report["phone_to_lidar_offset_ns"] - true_offset), 5_000_000)
            self.assertEqual(report["lidar_clock_source"], 0)
            self.assertFalse(report["clock_alignment_verified"])
            original = report_path.read_bytes()
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(report_path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
