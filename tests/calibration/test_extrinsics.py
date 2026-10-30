"""Measured-pair solver checks with a known synthetic rigid transform."""
from contextlib import redirect_stderr, redirect_stdout
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "calibration"))
from solve_extrinsics import CalibrationError, main, parse_observations, solve_document


def measured_example(noise=0.25, bad_pairs=True):
    rng = np.random.default_rng(19)
    rotation = Rotation.from_euler("xyz", [11.0, -23.0, 151.0], degrees=True).as_matrix()
    translation = np.array([0.061, -0.037, 0.091])
    document = {"schema_version": 1, "camera_lens": "wide",
                "image_coordinates": "native_unrotated", "views": []}
    # Synthetic points on each board are coplanar; different board poses
    # collectively constrain a single 6-DoF extrinsic transform.
    board = np.array([[x, y, 0.0] for x in [-0.65, -0.2, 0.2, 0.65] for y in [-0.5, 0, 0.5]])
    for i in range(10):
        k = np.array([[890 + 42 * i, 0, 960 + 2 * i],
                      [0, 950 - 13 * i, 720 - i], [0, 0, 1]], dtype=float)
        board_rotation = Rotation.from_euler("xyz", [(-1) ** i * 13, i * 3 - 10, i * 2], degrees=True).as_matrix()
        camera = board @ board_rotation.T + [0.13 * (i % 3 - 1), 0.1 * (i % 2), 2.2 + 0.3 * i]
        xyz = (camera - translation) @ rotation
        uvh = camera @ k.T
        uv = uvh[:, :2] / uvh[:, 2:]
        uv += rng.normal(0, noise, uv.shape)
        if bad_pairs and i < 7:
            uv[2] += [65, -48]
            uv[7] += [-52, 71]
        document["views"].append({
            "id": f"pose-{i:02d}", "split": "train" if i < 7 else "validation",
            "static_pair_confirmed": True,
            "image": f"{i:08d}.jpg", "width": 1920, "height": 1440,
            "intrinsics_row_major": k.ravel().tolist(),
            "correspondences": [{"id": f"corner-{j}", "xyz_lidar_m": p.tolist(), "uv_px": pixel.tolist()}
                                for j, (p, pixel) in enumerate(zip(xyz, uv))]})
    return document, rotation, translation


class ExtrinsicTests(unittest.TestCase):
    def test_recovers_transform_with_per_frame_intrinsics_noise_and_bad_pairs(self):
        document, expected_rotation, expected_translation = measured_example()
        result = solve_document(document)
        self.assertFalse(result["verified"])
        self.assertTrue(result["quality_gates_passed"], result["quality_gates"])
        actual_rotation = np.array(result["opencv_transform"]["rotation_matrix_row_major"]).reshape(3, 3)
        actual_translation = np.array(result["opencv_transform"]["translation_m"])
        rotation_error = Rotation.from_matrix(actual_rotation @ expected_rotation.T).magnitude()
        self.assertLess(np.degrees(rotation_error), 0.08)
        self.assertLess(np.linalg.norm(actual_translation - expected_translation), 0.005)
        self.assertLess(result["validation_all_pairs"]["p95_px"], 0.9)
        self.assertGreater(result["training_all_pairs"]["max_px"], 60)
        self.assertEqual(result["training_fit_inliers"]["count"], 70)
        self.assertEqual(result["validation_all_pairs"]["count"], 36)
        # The proposal contains finite values only, including failure diagnostics.
        json.dumps(result, allow_nan=False)

    def test_arkit_axes_and_camera_center(self):
        document, expected_rotation, expected_translation = measured_example(noise=0, bad_pairs=False)
        result = solve_document(document)
        axes = np.diag([1, -1, -1])
        actual_rotation = np.array(result["lidar_to_camera"]["rotation_matrix_row_major"]).reshape(3, 3)
        actual_translation = np.array(result["lidar_to_camera"]["translation_m"])
        np.testing.assert_allclose(actual_rotation, axes @ expected_rotation, atol=1e-7)
        np.testing.assert_allclose(actual_translation, axes @ expected_translation, atol=1e-7)
        np.testing.assert_allclose(result["camera_center_lidar_m"], -expected_rotation.T @ expected_translation, atol=1e-7)
        self.assertAlmostEqual(np.linalg.det(actual_rotation), 1.0)

    def test_held_out_pairs_do_not_influence_fit_or_get_discarded(self):
        document, _, _ = measured_example()
        original = solve_document(document)
        for view in document["views"]:
            if view["split"] == "validation":
                for pair in view["correspondences"]:
                    pair["uv_px"][0] += 25
        rejected = solve_document(document)
        self.assertEqual(original["lidar_to_camera"], rejected["lidar_to_camera"])
        self.assertFalse(rejected["quality_gates_passed"])
        self.assertFalse(rejected["verified"])
        self.assertEqual(rejected["validation_all_pairs"]["count"], 36)
        self.assertFalse(rejected["quality_gates"]["held_out_p95"]["passed"])
        self.assertTrue(all(not pair["used_for_fit"] for view in rejected["views"]
                            if view["split"] == "validation" for pair in view["pairs"]))

    def test_requires_independent_held_out_views(self):
        document, _, _ = measured_example()
        document["views"] = document["views"][:8]
        with self.assertRaisesRegex(CalibrationError, "3 vistas train y 2 validation"):
            solve_document(document)

    def test_rejects_copied_training_pose_in_validation(self):
        document, _, _ = measured_example()
        copied = copy.deepcopy(document["views"][0])
        copied.update(id="copied", split="validation")
        document["views"][-1] = copied
        with self.assertRaisesRegex(CalibrationError, "repite"):
            parse_observations(document)

    def test_rejects_single_plane_training_set(self):
        document, _, _ = measured_example()
        for view in document["views"]:
            if view["split"] == "train":
                for pair in view["correspondences"]:
                    pair["xyz_lidar_m"][2] = 1.25
        with self.assertRaisesRegex(CalibrationError, "degenerado"):
            solve_document(document)

    def test_rejects_collinear_view(self):
        document, _, _ = measured_example()
        for j, pair in enumerate(document["views"][0]["correspondences"]):
            pair["xyz_lidar_m"] = [j * 0.1, 0, 1]
        with self.assertRaisesRegex(CalibrationError, "colineales"):
            solve_document(document)

    def test_rejects_malformed_coordinates_and_intrinsics(self):
        baseline, _, _ = measured_example()
        for mutation in [
            lambda d: d.update(image_coordinates="portrait_display"),
            lambda d: d["views"][0].update(static_pair_confirmed=False),
            lambda d: d["views"][0]["correspondences"][0].update(uv_px=[1920, 5]),
            lambda d: d["views"][0]["correspondences"][0].update(xyz_lidar_m=[float("nan"), 0, 1]),
            lambda d: d["views"][0].update(intrinsics_row_major=[1] * 9),
            lambda d: d["views"][0].update(split="test"),
        ]:
            with self.subTest(mutation=mutation):
                document = copy.deepcopy(baseline)
                mutation(document)
                with self.assertRaises(CalibrationError):
                    solve_document(document)

    def test_requires_explicit_static_pair_confirmation(self):
        document, _, _ = measured_example()
        del document["views"][0]["static_pair_confirmed"]
        with self.assertRaisesRegex(CalibrationError, "static_pair_confirmed"):
            solve_document(document)

    def test_cli_writes_new_report_and_refuses_to_overwrite(self):
        document, _, _ = measured_example()
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "pairs.json"
            destination = Path(temporary) / "proposal.json"
            source.write_text(json.dumps(document))
            argv = [str(source), "--output", str(destination)]
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(main(argv), 0)
                first = destination.read_bytes()
                self.assertEqual(main(argv), 1)
                self.assertEqual(first, destination.read_bytes())
            report = json.loads(first)
            self.assertEqual(len(report["source"]["sha256"]), 64)
            self.assertFalse(report["verified"])

    def test_cli_quality_failure_is_saved_as_unverified_and_returns_two(self):
        document, _, _ = measured_example()
        document["views"][-1]["correspondences"][0]["uv_px"][0] += 80
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "pairs.json"
            destination = Path(temporary) / "rejected.json"
            source.write_text(json.dumps(document))
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main([str(source), "--output", str(destination)]), 2)
            report = json.loads(destination.read_text())
            self.assertFalse(report["quality_gates_passed"])
            self.assertFalse(report["verified"])


if __name__ == "__main__":
    unittest.main()
