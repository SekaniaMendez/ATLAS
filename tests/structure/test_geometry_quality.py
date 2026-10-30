"""Independent checks of measurement coverage and source-retention decisions."""
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "structure"))
from cloud_io import load_cloud
from mesh_quality import measure_quality
from scene_report import retention_decision


class GeometryQualityTest(unittest.TestCase):
    def setUp(self):
        grid = np.linspace(-1, 1, 41)
        self.points = np.array([[x, y, 0] for x in grid for y in grid])
        self.patch = {"origin": np.zeros(3), "axes": np.eye(3),
                      "vertices": np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]]),
                      "triangles": np.array([[0, 1, 2], [0, 2, 3]])}
        self.metadata = {"gravity_aligned": True,
                         "slam_session": {"tracking_state": "2", "fatal_error": "0"}}

    def test_plane_distance_and_reference_transform(self):
        # Independent rigid transform of both measurements and model.
        axes = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        origin = np.array([2.4, -1.8, 3.2])
        self.patch.update(axes=axes, origin=origin)
        measured = np.einsum("nj,ij->ni", self.points, axes) + origin
        quality = measure_quality(measured, [self.patch])
        self.assertEqual(quality["point_coverage"], 1)
        self.assertLess(quality["point_to_mesh_max_m"], 1e-5)
        self.assertEqual(quality["unsupported_area_fraction"], 0)
        self.assertTrue(retention_decision(quality, self.metadata, 5)[0])

    def test_filled_hole_fails_even_when_every_point_is_close(self):
        ring = self.points[np.max(np.abs(self.points[:, :2]), axis=1) > .5]
        quality = measure_quality(ring, [self.patch])
        self.assertEqual(quality["point_coverage"], 1)
        self.assertGreater(quality["unsupported_area_fraction"], .1)
        self.assertFalse(retention_decision(quality, self.metadata, 5)[0])

    def test_missing_details_tracking_and_size_preserve_source(self):
        measured = np.concatenate([self.points, self.points[:100] + [0, 0, .2]])
        quality = measure_quality(measured, [self.patch])
        self.assertLess(quality["point_coverage"], .99)
        self.assertFalse(retention_decision(quality, self.metadata, 5)[0])
        quality = measure_quality(self.points, [self.patch])
        self.assertFalse(retention_decision(quality, self.metadata, 1.5)[0])
        self.assertFalse(retention_decision(quality, {}, 5)[0])
        self.metadata["slam_session"]["tracking_state"] = "3"
        self.assertFalse(retention_decision(quality, self.metadata, 5)[0])

    def test_truncated_nonfinite_or_misdeclared_ply_rejected(self):
        header = ("ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\n"
                  "property float y\nproperty float z\nend_header\n")
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "map.ply"
            for body in ("0 0 0\n", "0 0 0\n1 0 0\nnan 1 0\n", "0 0 0\n1 0 0\n0 1 0\n2 2 2\n"):
                path.write_text(header + body)
                with self.assertRaises(ValueError):
                    load_cloud(path)
            path.write_text(header + "0 0 0\n1 0 0\n0 1 0\n")
            points, metadata = load_cloud(path)
            self.assertEqual(points.shape, (3, 3))
            self.assertEqual(metadata["input_points"], 3)

    def test_invalid_metrics_never_authorize_discard(self):
        valid = {"point_coverage": 1.0, "unsupported_area_fraction": 0.0}
        for field in valid:
            for value in (float("nan"), float("inf"), -.1, 1.1, None):
                with self.subTest(field=field, value=value):
                    self.assertFalse(retention_decision({**valid, field: value}, self.metadata, 5)[0])
        for ratio in (float("nan"), float("inf"), 0, -1, None):
            self.assertFalse(retention_decision(valid, self.metadata, ratio)[0])


if __name__ == "__main__":
    unittest.main()
