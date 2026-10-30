"""Check the pairing workspace keeps raw coordinates and rejects bad metadata."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "calibration"))
from prepare_workspace import prepare


class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.lidar = self.root / "lidar"
        self.camera = self.root / "phone"
        self.lidar.mkdir()
        self.camera.mkdir()
        self.cloud = np.array([[1, 2, 3], [1, 2.5, 3.5], [2, 2, 3]], dtype=float)
        header = "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
        (self.lidar / "cloud.ply").write_text(header + "\n".join(" ".join(map(str, p)) for p in self.cloud) + "\n")
        self.metadata = {"coordinate_frame": "lidar_sensor", "units": "m", "arrival_start_s": 2, "duration_s": 5}
        self.frame = {"image": "00000000.jpg", "width": 20, "height": 16,
                      "intrinsics_row_major": [30, 0, 10, 0, 31, 8, 0, 0, 1], "timestamp_s": 123.45}
        _, jpeg = cv2.imencode(".jpg", np.zeros((16, 20, 3), dtype=np.uint8))
        (self.camera / "00000000.jpg").write_bytes(jpeg.tobytes())
        self.session = {"lens": "wide", "image_coordinates": "native unrotated camera image"}
        self.spec = {"camera_lens": "wide", "views": [{"id": "pose-01", "split": "train",
                                                      "lidar_directory": "lidar", "camera_frame": "phone/00000000.json"}]}
        self.manifest = self.root / "manifest.json"
        self.output = self.root / "prepared"
        self.write_inputs()

    def write_inputs(self):
        for path, value in [(self.lidar / "metadata.json", self.metadata),
                            (self.camera / "00000000.json", self.frame),
                            (self.camera / "session.json", self.session),
                            (self.manifest, self.spec)]:
            path.write_text(json.dumps(value))

    def test_prepares_relative_paths_raw_points_and_provenance(self):
        result = prepare(self.manifest, self.output)
        view = result["views"][0]
        self.assertFalse(view["static_pair_confirmed"])
        self.assertEqual(view["correspondences"], [])
        self.assertEqual(view["intrinsics_row_major"], self.frame["intrinsics_row_major"])
        self.assertEqual(view["camera_timestamp_s"], self.frame["timestamp_s"])
        self.assertEqual(result["image_coordinates"], "native_unrotated")
        points = np.fromfile(self.output / "pose-01" / "points.f32", dtype="<f4").reshape(-1, 3)
        np.testing.assert_allclose(points, np.unique(self.cloud, axis=0))
        original_image = (self.camera / "00000000.jpg").read_bytes()
        self.assertEqual((self.output / "pose-01" / "image.jpg").read_bytes(), original_image)
        self.assertEqual(view["image_sha256"], hashlib.sha256(original_image).hexdigest())
        self.assertTrue((self.output / "collector.html").is_file())
        self.assertTrue((self.output / "collector.js").is_file())
        self.assertEqual(json.loads((self.output / "observations.json").read_text()), result)

    def test_rejects_wrong_units_or_world_map_before_writing(self):
        for field, value in [("units", "mm"), ("coordinate_frame", "slam_world")]:
            with self.subTest(field=field):
                original = copy.deepcopy(self.metadata)
                self.metadata[field] = value
                self.write_inputs()
                with self.assertRaisesRegex(ValueError, "coordenadas del sensor"):
                    prepare(self.manifest, self.output)
                self.assertFalse(self.output.exists())
                self.metadata = original

    def test_rejects_wrong_lens_in_manifest_or_camera_session(self):
        self.spec["camera_lens"] = "ultraWide"
        self.write_inputs()
        with self.assertRaisesRegex(ValueError, "wide"):
            prepare(self.manifest, self.output)
        self.spec["camera_lens"] = "wide"
        self.session["lens"] = "ultraWide"
        self.write_inputs()
        with self.assertRaisesRegex(ValueError, "wide"):
            prepare(self.manifest, self.output)
        self.assertFalse(self.output.exists())

    def test_existing_output_is_not_modified(self):
        self.output.mkdir()
        sentinel = self.output / "keep.txt"
        sentinel.write_text("keep this content")
        with self.assertRaises(FileExistsError):
            prepare(self.manifest, self.output)
        self.assertEqual(list(self.output.iterdir()), [sentinel])
        self.assertEqual(sentinel.read_text(), "keep this content")

    def test_accepts_legacy_wide_lens_display_label(self):
        self.session["lens"] = "1×"
        self.write_inputs()
        self.assertEqual(prepare(self.manifest, self.output)["camera_lens"], "wide")

    def test_rejects_invalid_k_dimensions_and_time_before_creating_output(self):
        original = copy.deepcopy(self.frame)
        invalid = [
            {"intrinsics_row_major": [1, 2]},
            {"intrinsics_row_major": [30, 0, 10, 0, 31, 8, 0, 0, 2]},
            {"intrinsics_row_major": [30, 1, 10, 0, 31, 8, 0, 0, 1]},
            {"intrinsics_row_major": [30, 0, 20, 0, 31, 8, 0, 0, 1]},
            {"intrinsics_row_major": [-30, 0, 10, 0, 31, 8, 0, 0, 1]},
            {"intrinsics_row_major": [float("nan"), 0, 10, 0, 31, 8, 0, 0, 1]},
            {"width": 20.5}, {"height": True}, {"width": -3},
            {"timestamp_s": float("nan")}, {"timestamp_s": -1},
        ]
        for change in invalid:
            with self.subTest(change=change):
                self.frame = {**original, **change}
                self.write_inputs()
                with self.assertRaises(ValueError):
                    prepare(self.manifest, self.output)
                self.assertFalse(self.output.exists())

    def test_rejects_image_path_escape(self):
        self.frame["image"] = "../elsewhere.jpg"
        self.write_inputs()
        with self.assertRaisesRegex(ValueError, "misma carpeta"):
            prepare(self.manifest, self.output)
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
