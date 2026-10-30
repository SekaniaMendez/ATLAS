"""Regression checks for observed support, orientation and frame observability."""
import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np
from shapely.geometry import Polygon

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "structure"))
from plane_geometry import extract, derive_structural_frame
from structure_config import StructureConfig


def horizontal(x0=0, x1=2, y0=0, y1=2, z=0, step=0.04):
    x, y = np.meshgrid(np.arange(x0, x1 + step / 2, step),
                       np.arange(y0, y1 + step / 2, step))
    return np.column_stack([x.ravel(), y.ravel(), np.full(x.size, z)])


class PlaneGeometryTest(unittest.TestCase):
    config = replace(StructureConfig(), min_support_points=50, ransac_iterations=400,
                     max_plane_hypotheses=8)

    def test_measured_hole_is_not_filled_and_planes_compress(self):
        points = horizontal()
        gap = ((points[:, 0] > 0.6) & (points[:, 0] < 1.4)
               & (points[:, 1] > 0.6) & (points[:, 1] < 1.4))
        points = points[~gap]
        original = points.copy()
        patches = extract(points, self.config)
        self.assertGreaterEqual(len(patches), 1)
        self.assertTrue(np.array_equal(points, original), "extraction modified measurements")
        # Leave the alpha interpolation margin outside this interior hole.
        interior = Polygon([(0.75, 0.75), (1.25, 0.75), (1.25, 1.25), (0.75, 1.25)])
        face_count = 0
        for patch in patches:
            world = patch["vertices"] @ patch["axes"].T + patch["origin"]
            for tri in patch["triangles"]:
                self.assertLess(Polygon(world[tri, :2]).intersection(interior).area, 1e-10)
            face_count += len(patch["triangles"])
            self.assertEqual(patch["kind"], "horizontal")
            self.assertLess(patch["rmse_m"], 1e-8)
            self.assertTrue(np.allclose(patch["axes"].T @ patch["axes"], np.eye(3)))
            self.assertAlmostEqual(np.linalg.det(patch["axes"]), 1.0)
        self.assertLess(face_count, len(points) // 5, "plane representation failed to compact support")

    def test_disconnected_coplanar_patches_do_not_bridge_space(self):
        points = np.concatenate([horizontal(0, 1, 0, 1), horizontal(3, 4, 0, 1)])
        patches = extract(points, self.config)
        self.assertEqual(len(patches), 2)
        assignments = np.concatenate([patch["support_indices"] for patch in patches])
        self.assertEqual(len(assignments), len(np.unique(assignments)))
        for patch in patches:
            world = patch["vertices"] @ patch["axes"].T + patch["origin"]
            self.assertLess(np.ptp(world[:, 0]), 1.01)

    def test_gravity_and_yaw_are_not_invented(self):
        patches = extract(horizontal(), self.config)
        self.assertIsNone(derive_structural_frame(patches))
        frame = derive_structural_frame(patches, gravity_aligned=True)
        self.assertFalse(frame["yaw_defined_by_plane"])
        self.assertFalse(frame["heading_is_geographic"])
        self.assertTrue(np.allclose(frame["axes"], np.eye(3)))

    def test_oblique_wall_preserves_measured_orientation(self):
        angle = np.deg2rad(31)
        wall = horizontal(0, 2, 0, 2)
        u, v = wall[:, 0].copy(), wall[:, 1].copy()
        wall[:, 0] = np.cos(angle) * u
        wall[:, 1] = np.sin(angle) * u
        wall[:, 2] = v
        rng = np.random.default_rng(10)
        normal = np.array([-np.sin(angle), np.cos(angle), 0])
        wall += rng.normal(0, 0.003, (len(wall), 1)) * normal
        patches = extract(wall, self.config)
        largest = max(patches, key=lambda patch: patch["area_m2"])
        self.assertEqual(largest["kind"], "vertical")
        self.assertGreater(abs(largest["normal"] @ normal), 0.999)
        self.assertLess(largest["rmse_m"], 0.006)
        self.assertLessEqual(largest["max_residual_m"], self.config.distance_threshold_m + 1e-12)
        frame = derive_structural_frame(patches, gravity_aligned=True)
        self.assertTrue(frame["yaw_defined_by_plane"])
        self.assertGreater(abs(frame["axes"][:, 0] @ normal), 0.999)
        self.assertTrue(np.allclose(frame["axes"][:, 2], [0, 0, 1]))
        transform = frame["T_map_structural"]
        self.assertTrue(np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3)))
        self.assertAlmostEqual(np.linalg.det(transform[:3, :3]), 1.0)

    def test_input_order_does_not_change_patch_identity(self):
        points = np.concatenate([horizontal(0, 1, 0, 1), horizontal(3, 4, 0, 1)])
        first = extract(points, self.config)
        second = extract(points[::-1], self.config)
        self.assertEqual([patch["id"] for patch in first], [patch["id"] for patch in second])
        for a, b in zip(first, second):
            self.assertTrue(np.allclose(a["origin"], b["origin"]))
            self.assertTrue(np.allclose(a["normal"], b["normal"]))
            self.assertAlmostEqual(a["area_m2"], b["area_m2"])

    def test_degenerate_and_invalid_inputs(self):
        self.assertEqual(extract(np.empty((0, 3)), self.config), [])
        with self.assertRaises(ValueError):
            extract(np.array([[np.nan, 0, 0]]), self.config)
        with self.assertRaises(ValueError):
            extract(np.zeros((100, 2)), self.config)
        with self.assertRaises(ValueError):
            replace(self.config, distance_threshold_m=-1).validate()
        line = np.column_stack([np.linspace(0, 2, 100), np.zeros((100, 2))])
        self.assertEqual(extract(line, self.config), [])


if __name__ == "__main__":
    unittest.main()
