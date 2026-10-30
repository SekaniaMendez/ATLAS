"""Regressions for measured freeform surfaces and conservative simplification."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools' / 'structure'))
from mesh_quality import distance_to_mesh, measure_quality, world_mesh
from residual_geometry import reconstruct_residual, _mesh, _simplify_bounded, _bounded_faces


def grid_mesh():
    n = 18
    vertices = np.array([[x * .04, y * .04, .2] for x in range(n) for y in range(n)])
    faces = []
    for x in range(n - 1):
        for y in range(n - 1):
            a, b = x * n + y, (x + 1) * n + y
            faces.extend([[a, b, a + 1], [a + 1, b, b + 1]])
    return vertices, _mesh(vertices, faces)


class ResidualGeometryTest(unittest.TestCase):
    def test_hole_and_isolated_measurements_stay_unmodeled(self):
        axis = np.linspace(-.8, .8, 41)
        points = np.array([[x, y, .2 + .02 * (x*x + y*y)] for x in axis for y in axis
                           if max(abs(x), abs(y)) > .26])
        measured_count = len(points)
        points = np.concatenate([points, [[4, 4, 4], [-4, 3, 3]]])
        original = points.copy()
        patches = reconstruct_residual(points, [])
        np.testing.assert_array_equal(points, original)
        self.assertEqual(len(patches), 1)
        self.assertTrue(np.all(patches[0]['support_indices'] < measured_count))
        self.assertGreater(distance_to_mesh(np.array([[0., 0., .2]]), patches)[0], .20)
        self.assertTrue(np.all(distance_to_mesh(points[-2:], patches) > 1))
        self.assertGreater((distance_to_mesh(points[:measured_count], patches) <= .04).mean(), .98)

    def test_curved_surface_coverage_bounds_and_unique_faces(self):
        points = np.array([[.6 * np.cos(a), .6 * np.sin(a), z]
                           for a in np.linspace(-1.2, 1.2, 51)
                           for z in np.linspace(0, 1, 26)])
        patches = reconstruct_residual(points, [])
        quality = measure_quality(points, patches)
        self.assertGreater(quality['point_coverage'], .99)
        self.assertEqual(quality['unsupported_area_fraction'], 0)
        mesh = world_mesh(patches)
        vertices, faces = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
        self.assertEqual(len(np.unique(np.sort(faces, axis=1), axis=0)), len(faces))
        self.assertTrue(_bounded_faces(vertices, faces, .2, cKDTree(points)).all())
        support = patches[0]['support_indices']
        self.assertTrue(np.all(distance_to_mesh(points[support], patches) <= .040001))
        info = patches[0]['simplification']
        if info['accepted']:
            self.assertLessEqual(info['relative_area_change'], .01)
            self.assertLessEqual(info['reference_to_reduced_max_probe_m'], .015001)
            self.assertLessEqual(info['reduced_to_reference_max_probe_m'], .015001)

    def test_bad_decimation_falls_back_to_measured_surface(self):
        points, reference = grid_mesh()
        bad = _mesh(np.asarray(reference.vertices) + [0, 0, .5], np.asarray(reference.triangles))
        with patch('residual_geometry._decimate', return_value=bad):
            result, info = _simplify_bounded(reference, points, np.ones(len(points), bool), .04, .2)
        self.assertFalse(info['accepted'])
        np.testing.assert_array_equal(np.asarray(result.vertices), np.asarray(reference.vertices))
        np.testing.assert_array_equal(np.asarray(result.triangles), np.asarray(reference.triangles))

    def test_reversed_duplicates_and_degenerate_faces_removed(self):
        result = _mesh([[0, 0, 0], [.1, 0, 0], [0, .1, 0]],
                       [[0, 1, 2], [2, 1, 0], [1, 2, 0], [0, 0, 1]])
        self.assertEqual(len(result.triangles), 1)
        self.assertAlmostEqual(result.get_surface_area(), .005)

    def test_no_artificial_surface_for_isolated_or_invalid_points(self):
        points = np.array([[0., 0., 0.], [2., 0., 1.], [0., 2., 3.], [5., 2., 1.]])
        self.assertEqual(reconstruct_residual(points, []), [])
        self.assertEqual(reconstruct_residual(np.empty((0, 3)), []), [])
        for values in (np.zeros((4, 2)), np.array([[0, 0, np.nan]])):
            with self.assertRaises(ValueError):
                reconstruct_residual(values, [])
        with self.assertRaises(ValueError):
            reconstruct_residual(points, [], max_edge_m=-1)


if __name__ == '__main__':
    unittest.main()
