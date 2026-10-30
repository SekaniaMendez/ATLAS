"""Snapshot identity and preservation of measurements and existing labels."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "structure"))
import prepare_annotations as preparation


class PrepareAnnotationsTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.capture = Path(self.temporary.name) / "capture-a"
        self.capture.mkdir()
        self.source = self.capture / "mapa.ply"
        # Include a repeated measurement: manifest declares canonical unique XYZ.
        self.source.write_text("ply\nformat ascii 1.0\nelement vertex 4\n"
                               "property float x\nproperty float y\nproperty float z\n"
                               "end_header\n0 0 0\n1 1 1\n2 2 2\n0 0 0\n")
        self.source_bytes = self.source.read_bytes()
        self.destination = self.capture / "anotaciones"

    def test_reuse_preserves_labels_and_source_and_canonical_points(self):
        with patch.object(preparation, "extract", return_value=[]):
            destination = preparation.prepare(self.source)
        manifest = json.loads((destination / "manifest.json").read_text())
        self.assertEqual(manifest["point_count"], 3)
        self.assertEqual(manifest["source"]["input_points"], 4)
        self.assertEqual(manifest["source"]["point_order"], "lexicographic_unique_xyz")
        self.assertEqual(manifest["input_features"], ["x", "y", "z"])
        with np.load(destination / "points.npz", allow_pickle=False) as arrays:
            self.assertEqual(arrays["points"].shape, (3, 3))
            self.assertEqual(len(arrays["point_group_ids"]), 3)
        label_file = destination / "annotations.json"
        label_file.write_text('{"human_work":"do not replace"}')
        label_bytes = label_file.read_bytes()
        with patch.object(preparation, "extract", side_effect=AssertionError("must not recompute")):
            self.assertEqual(preparation.prepare(self.source, reuse=True), destination)
        self.assertEqual(label_file.read_bytes(), label_bytes)
        self.assertEqual(self.source.read_bytes(), self.source_bytes)
        with self.assertRaisesRegex(ValueError, "destino ya existe"):
            preparation.prepare(self.source)

    def test_changed_source_or_session_never_reuses_labels(self):
        with patch.object(preparation, "extract", return_value=[]):
            preparation.prepare(self.source)
        (self.capture / "slam_session.txt").write_text("scans_accepted=1\n")
        with self.assertRaisesRegex(ValueError, "han cambiado"):
            preparation.prepare(self.source, reuse=True)
        (self.capture / "slam_session.txt").unlink()
        self.source.write_bytes(self.source_bytes.replace(b"2 2 2", b"3 3 3"))
        with self.assertRaisesRegex(ValueError, "han cambiado"):
            preparation.prepare(self.source, reuse=True)

    def test_corrupt_points_and_invalid_partition_are_rejected(self):
        broken = {"groups": [{"point_count": 1}], "point_group_ids": np.zeros(3),
                  "diagnostics": {}}
        with patch.object(preparation, "extract", return_value=[]), \
             patch.object(preparation, "build_groups", return_value=broken):
            with self.assertRaisesRegex(ValueError, "conserva exactamente"):
                preparation.prepare(self.source)
        self.assertFalse(self.destination.exists())
        self.assertEqual(self.source.read_bytes(), self.source_bytes)
        with patch.object(preparation, "extract", return_value=[]):
            preparation.prepare(self.source)
        (self.destination / "points.npz").write_bytes(b"corrupted")
        with self.assertRaisesRegex(ValueError, "han cambiado"):
            preparation.prepare(self.source, reuse=True)


if __name__ == "__main__":
    unittest.main()
