"""Human labels stay attached to measured points and survive safe reloads."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "structure"))
from annotation_store import AnnotationError, AnnotationStore, DatasetChanged, RevisionConflict
from category_registry import CategoryRegistry, CategoryRegistryError


def create_dataset(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, .5], [0, 1, 1],
                       [1, 1, .5], [1, 1, 1], [4, 4, 4]], dtype=np.float32)
    np.savez_compressed(directory / "points.npz", points=points,
                        point_group_ids=np.array([0, 0, 1, 1, 2, 2, 3], dtype=np.int32))
    manifest = {"schema_version": 1, "dataset_id": "capture-test-a1",
                "source": {"source_name": "mapa.ply", "source_sha256": "a" * 64,
                           "capture_id": "capture-test", "point_order": "lexicographic_unique_xyz"},
                "point_count": len(points), "units": "metres", "map_frame": "slam_map",
                "points_sha256": hashlib.sha256((directory / "points.npz").read_bytes()).hexdigest(),
                "groups": [
                    {"id": "floor_a", "kind": "structure", "suggested_label": "floor", "point_count": 2},
                    {"id": "object_a", "kind": "object", "suggested_label": None, "point_count": 2},
                    {"id": "object_b", "kind": "object", "suggested_label": None, "point_count": 2},
                    {"id": "unassigned", "kind": "unassigned", "suggested_label": None, "point_count": 1},
                ]}
    (directory / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return directory


class AnnotationStoreTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = create_dataset(Path(self.temp.name) / "dataset")
        self.store = AnnotationStore(self.directory)
        self.initial_bytes = {name: (self.directory / name).read_bytes()
                              for name in ("manifest.json", "points.npz")}

    def tearDown(self):
        for name, original in self.initial_bytes.items():
            if name not in getattr(self, "mutated", set()):
                self.assertEqual((self.directory / name).read_bytes(), original)
        self.temp.cleanup()

    def label(self, revision=0, ids=None, label="chair"):
        return self.store.update({"revision": revision, "group_ids": ids or ["object_a"],
                                  "action": "label", "label": label, "notes": "revisión manual"})

    def test_suggestions_are_not_labels_and_export_keeps_unknown_points(self):
        description = self.store.describe()
        self.assertEqual(description["manifest"]["groups"][0]["suggested_label"], "floor")
        self.assertEqual(description["annotations"]["groups"]["floor_a"]["status"], "pending")
        result = self.store.export(0)
        self.assertEqual(result["summary"]["reviewed_points"], 0)
        with np.load(self.store.export_file("revision-000000.npz"), allow_pickle=False) as data:
            np.testing.assert_array_equal(data["points"], self.store.points)
            self.assertTrue(np.all(data["semantic_labels"] == -1))
            self.assertFalse(data["reviewed_mask"].any())

    def test_group_merge_label_reload_and_pointwise_training_export(self):
        labeled = self.label(ids=["object_a", "object_b"])
        self.assertEqual(labeled["revision"], 1)
        self.assertEqual(labeled["groups"]["object_a"]["instance_id"], labeled["groups"]["object_b"]["instance_id"])
        self.store.update({"revision": 1, "action": "ignore", "group_ids": ["unassigned"]})
        reloaded = AnnotationStore(self.directory)
        self.assertEqual(reloaded.read()["groups"]["object_a"]["label"], "chair")
        result = reloaded.export(2)
        self.assertEqual(result["summary"]["reviewed_points"], 4)
        self.assertEqual(result["summary"]["ignored_points"], 1)
        self.assertEqual(result["summary"]["pending_points"], 2)
        self.assertEqual(result["summary"]["instances"], 1)
        with np.load(reloaded.export_file("revision-000002.npz"), allow_pickle=False) as data:
            np.testing.assert_array_equal(data["semantic_labels"], [-1, -1, 3, 3, 3, 3, -1])
            np.testing.assert_array_equal(data["instance_ids"], [-1, -1, 1, 1, 1, 1, -1])
            np.testing.assert_array_equal(data["reviewed_mask"], [False, False, True, True, True, True, False])
            self.assertTrue(data["ignored_mask"][-1])
            np.testing.assert_array_equal(data["canonical_point_indices"], np.arange(7))
        metadata = json.loads(reloaded.export_file("revision-000002.json").read_text())
        self.assertEqual(metadata["split"], "unassigned")
        self.assertIsNone(metadata["location_id"])
        self.assertTrue(metadata["requires_location_id_before_train_validation_split"])
        self.assertFalse(metadata["automatic_suggestions_are_targets"])
        self.assertEqual(metadata["source_sha256"], "a" * 64)
        # Re-exporting the same revision is idempotent and preserves all bytes.
        before = reloaded.export_file("revision-000002.npz").read_bytes()
        self.assertEqual(reloaded.export(2), result)
        self.assertEqual(reloaded.export_file("revision-000002.npz").read_bytes(), before)

    def test_reset_excludes_points_without_deleting_previous_export(self):
        self.label()
        self.store.export(1)
        old_export = self.store.export_file("revision-000001.npz").read_bytes()
        self.store.update({"revision": 1, "group_ids": ["object_a"], "action": "reset"})
        self.assertEqual(self.store.export(2)["summary"]["reviewed_points"], 0)
        self.assertEqual(self.store.export_file("revision-000001.npz").read_bytes(), old_export)

    def test_two_store_instances_cannot_overwrite_a_concurrent_revision(self):
        second = AnnotationStore(self.directory)
        request = {"revision": 0, "group_ids": ["object_a"], "action": "label", "label": "chair"}
        def update(store):
            try:
                return store.update(request)["revision"]
            except RevisionConflict as error:
                return error.code
        with ThreadPoolExecutor(max_workers=2) as pool:
            values = list(pool.map(update, (self.store, second)))
        self.assertCountEqual(values, [1, "revision_conflict"])
        with self.assertRaises(RevisionConflict):
            second.export(0)

    def test_failed_atomic_write_leaves_last_revision(self):
        before = self.store.annotations_path.read_bytes()
        with patch("annotation_store.os.replace", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.label()
        self.assertEqual(self.store.annotations_path.read_bytes(), before)
        self.assertEqual(self.store.read()["revision"], 0)
        self.assertEqual(list(self.directory.glob(".labels-*.tmp")), [])

    def test_invalid_edits_do_not_change_revision(self):
        requests = [
            {"revision": 0, "action": "label", "group_ids": ["object_a"], "label": "invented"},
            {"revision": 0, "action": "label", "group_ids": ["object_a", "object_a"], "label": "chair"},
            {"revision": 0, "action": "label", "group_ids": ["../other"], "label": "chair"},
            {"revision": True, "action": "reset", "group_ids": ["object_a"]},
            {"revision": 0, "action": "ignore", "group_ids": ["object_a"], "label": "chair"},
            {"revision": 0, "action": "reset", "group_ids": []},
            {"revision": 0, "action": "reset", "group_ids": ["object_a"], "notes": "x" * 501},
        ]
        for request in requests:
            with self.subTest(request=request), self.assertRaises(AnnotationError):
                self.store.update(request)
        self.assertEqual(self.store.read()["revision"], 0)

    def test_foreign_annotations_and_mutated_measurements_rejected(self):
        annotation = self.store.read()
        annotation["source_sha256"] = "b" * 64
        self.store.annotations_path.write_text(json.dumps(annotation))
        with self.assertRaises(DatasetChanged):
            AnnotationStore(self.directory)
        self.store.annotations_path.unlink()
        self.store = AnnotationStore(self.directory)
        self.mutated = {"points.npz"}
        np.savez_compressed(self.directory / "points.npz", points=self.store.points + 1,
                            point_group_ids=self.store.point_group_ids)
        with self.assertRaises(DatasetChanged):
            self.store.export(0)
        with self.assertRaises(DatasetChanged):
            AnnotationStore(self.directory)

    def test_invalid_memberships_and_duplicate_ids_fail_loading(self):
        self.mutated = {"manifest.json", "points.npz"}
        self.store.annotations_path.unlink()
        manifest = json.loads(self.store.manifest_path.read_text())
        manifest.pop("points_sha256")
        self.store.manifest_path.write_text(json.dumps(manifest))
        np.savez_compressed(self.store.points_path, points=self.store.points,
                            point_group_ids=np.array([0, 0, 1, 1, 2, 2, 99], dtype=np.int32))
        with self.assertRaises(AnnotationError):
            AnnotationStore(self.directory)
        self.store.points_path.write_bytes(self.initial_bytes["points.npz"])
        manifest["groups"][1]["id"] = manifest["groups"][0]["id"]
        self.store.manifest_path.write_text(json.dumps(manifest))
        with self.assertRaises(AnnotationError):
            AnnotationStore(self.directory)

    def test_exports_reject_traversal_symlinks_and_tampering(self):
        self.store.export(0)
        for name in ("../annotations.json", "revision-000000.json/..", "revision-0.npz", "data.npz"):
            with self.subTest(name=name), self.assertRaises(AnnotationError):
                self.store.export_file(name)
        exported = self.store.export_file("revision-000000.npz")
        exported.write_bytes(b"tampered")
        with self.assertRaises(DatasetChanged):
            self.store.export(0)
        exported.unlink()
        exported.symlink_to(self.store.points_path)
        with self.assertRaises(FileNotFoundError):
            self.store.export_file("revision-000000.npz")

    def create_object(self, indices, revision=0, label="chair"):
        return self.store.point_groups({"revision": revision, "point_indices": indices,
                                       "label": label, "notes": "dibujado con lápiz"})

    def test_manual_object_repartitions_measured_points_and_survives_reload(self):
        result = self.create_object([5, 2, 0])
        self.assertEqual(result["annotations"]["revision"], 1)
        self.assertEqual(result["manifest"]["partition_revision"], 1)
        groups = result["manifest"]["groups"]
        self.assertEqual([group["point_count"] for group in groups], [1, 1, 1, 1, 3])
        self.assertEqual(result["point_group_ids"], [4, 0, 4, 1, 2, 4, 3])
        manual = groups[-1]
        self.assertEqual(manual["id"], "manual-000001")
        self.assertTrue(manual["manual"])
        self.assertEqual(manual["kind"], "object")
        self.assertIsNone(manual["suggested_label"])
        self.assertEqual(manual["bounds_m"], [[0, 0, 0], [1, 1, 1]])
        np.testing.assert_allclose(manual["centroid_m"], self.store.points[[0, 2, 5]].mean(axis=0))
        self.assertEqual(result["annotations"]["groups"][manual["id"]]["label"], "chair")
        self.assertEqual(self.store.read()["manual_groups"][0]["point_indices"], [0, 2, 5])
        self.assertEqual(AnnotationStore(self.directory).describe(), result)
        np.testing.assert_array_equal(self.store.point_group_ids, [0, 0, 1, 1, 2, 2, 3])
        self.assertEqual(self.store.summary()["group_count"], 5)

    def test_manual_transfer_is_disjoint_retains_unselected_labels_and_restores_base(self):
        self.label(label="table")
        first = self.create_object([2, 3, 4], revision=1)
        self.assertNotIn("object_a", first["annotations"]["groups"])
        first_id = first["manifest"]["groups"][-1]["id"]
        second = self.create_object([3, 5], revision=2, label="sofa")
        second_id = second["manifest"]["groups"][-1]["id"]
        groups = {group["id"]: group for group in second["manifest"]["groups"]}
        self.assertEqual(groups[first_id]["point_count"], 2)
        self.assertEqual(groups[second_id]["point_count"], 2)
        self.store.export(3)
        with np.load(self.store.export_file("revision-000003.npz")) as data:
            np.testing.assert_array_equal(data["semantic_labels"], [-1, -1, 3, 5, 3, 5, -1])
            np.testing.assert_array_equal(data["instance_ids"], [-1, -1, 2, 3, 2, 3, -1])
            np.testing.assert_array_equal(data["base_point_group_ids"], self.store.point_group_ids)
            np.testing.assert_array_equal(data["points"], self.store.points)
        restored = self.store.point_groups({"revision": 3, "action": "remove", "group_id": second_id})
        self.assertEqual(restored["annotations"]["groups"]["object_a"]["label"], "table")
        self.assertEqual(restored["annotations"]["groups"][first_id]["label"], "chair")
        self.assertNotIn(second_id, restored["annotations"]["groups"])
        self.store.export(4)
        with np.load(self.store.export_file("revision-000004.npz")) as data:
            np.testing.assert_array_equal(data["semantic_labels"], [-1, -1, 3, 4, 3, -1, -1])

    def test_full_transfer_prunes_empty_groups_and_ids_are_never_reused(self):
        first = self.create_object(list(range(7)))
        self.assertEqual(len(first["manifest"]["groups"]), 1)
        self.assertEqual(first["point_group_ids"], [0] * 7)
        with self.assertRaises(AnnotationError):
            self.label(revision=1, ids=["object_a"])
        second = self.create_object(list(range(7)), revision=1, label="other")
        self.assertEqual([group["id"] for group in second["manifest"]["groups"]], ["manual-000002"])
        self.assertNotIn("manual-000001", self.store.read()["groups"])
        restored = self.store.point_groups({"revision": 2, "action": "remove", "group_id": "manual-000002"})
        self.assertEqual([group["id"] for group in restored["manifest"]["groups"]], self.store.group_ids)
        third = self.create_object([1], revision=3)
        self.assertEqual(third["manifest"]["groups"][-1]["id"], "manual-000003")

    def test_manual_labels_support_reset_ignore_and_human_only_exports(self):
        created = self.create_object([1, 3])
        group_id = created["manifest"]["groups"][-1]["id"]
        self.store.export(1)
        original = self.store.export_file("revision-000001.npz").read_bytes()
        ignored = self.store.update({"revision": 1, "group_ids": [group_id], "action": "ignore"})
        self.assertEqual(ignored["groups"][group_id]["status"], "ignored")
        self.assertEqual(self.store.export(2)["summary"]["ignored_points"], 2)
        reset = self.store.update({"revision": 2, "group_ids": [group_id], "action": "reset"})
        self.assertEqual(reset["groups"][group_id]["status"], "pending")
        self.assertEqual(self.store.export(3)["summary"]["reviewed_points"], 0)
        metadata = json.loads(self.store.export_file("revision-000003.json").read_text())
        self.assertEqual(metadata["manual_groups"][0]["point_indices"], [1, 3])
        self.assertEqual(metadata["manual_groups"][0]["provenance"], "human_selection")
        self.assertTrue(metadata["groups"][-1]["manual"])
        self.assertEqual(self.store.export_file("revision-000001.npz").read_bytes(), original)
        self.assertEqual(self.store.describe()["point_group_ids"], created["point_group_ids"])

    def test_manual_group_creation_conflicts_with_concurrent_label_edits(self):
        second = AnnotationStore(self.directory)
        def update(index):
            try:
                if index:
                    return second.point_groups({"revision": 0, "point_indices": [2], "label": "chair"})["annotations"]["revision"]
                return self.label()["revision"]
            except RevisionConflict as error:
                return error.code
        with ThreadPoolExecutor(max_workers=2) as pool:
            values = list(pool.map(update, (0, 1)))
        self.assertCountEqual(values, [1, "revision_conflict"])

    def test_invalid_point_selections_and_removals_do_not_mutate_state(self):
        bad_selections = [[], [0, 0], [-1], [7], [True], [0.0], ["1"], None, list(range(100001))]
        for selection in bad_selections:
            with self.subTest(selection=str(selection)[:80]), self.assertRaises(AnnotationError):
                self.create_object(selection)
        requests = [
            {"revision": 0, "point_indices": [0], "label": "invented"},
            {"revision": 0, "point_indices": [0], "label": "chair", "notes": "x" * 501},
            {"revision": True, "point_indices": [0], "label": "chair"},
            {"revision": 0, "action": "remove", "group_id": "object_a"},
            {"revision": 0, "action": "remove", "group_id": "manual-000001"},
            {"revision": 0, "action": "delete", "group_id": "object_a"},
        ]
        for request in requests:
            with self.subTest(request=request), self.assertRaises(AnnotationError):
                self.store.point_groups(request)
        self.assertEqual(self.store.read()["revision"], 0)
        self.create_object([0])
        with self.assertRaises(RevisionConflict):
            self.store.point_groups({"revision": 0, "action": "remove", "group_id": "manual-000001"})

    def test_manual_group_atomic_failure_preserves_previous_partition(self):
        self.create_object([1, 2])
        before = self.store.annotations_path.read_bytes()
        description = self.store.describe()
        with patch("annotation_store.os.replace", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.create_object([2, 3], revision=1)
        self.assertEqual(self.store.annotations_path.read_bytes(), before)
        self.assertEqual(self.store.describe(), description)
        self.assertEqual(list(self.directory.glob(".labels-*.tmp")), [])

    def test_loading_rejects_corrupt_manual_partition(self):
        self.create_object([0, 1])
        self.create_object([2, 3], revision=1)
        valid = self.store.read()
        for indices in ([0, 3], [3, 2], [2, 2], [7], [False]):
            annotation = json.loads(json.dumps(valid))
            annotation["manual_groups"][1]["point_indices"] = indices
            self.store.annotations_path.write_text(json.dumps(annotation))
            with self.subTest(indices=indices), self.assertRaises(AnnotationError):
                AnnotationStore(self.directory)

    def test_custom_category_ids_are_shared_across_captures_and_restarts(self):
        registry_path = Path(self.temp.name) / "categories.json"
        registry = CategoryRegistry(registry_path)
        category = registry.create("Estante")["category"]
        second_dir = create_dataset(Path(self.temp.name) / "dataset-second")
        second_manifest = json.loads((second_dir / "manifest.json").read_text())
        second_manifest["dataset_id"] = "capture-test-b2"
        (second_dir / "manifest.json").write_text(json.dumps(second_manifest))
        first = AnnotationStore(self.directory, registry=registry)
        second = AnnotationStore(second_dir, registry=CategoryRegistry(registry_path))
        first.update({"revision": 0, "action": "label", "label": category["key"], "group_ids": ["object_a"]})
        second.point_groups({"revision": 0, "point_indices": [0, 4], "label": category["key"]})
        for directory in (self.directory, second_dir):
            restarted = AnnotationStore(directory, registry=CategoryRegistry(registry_path))
            self.assertEqual(restarted.describe()["taxonomy"][-1], category)
            restarted.export(1)
            with np.load(restarted.export_file("revision-000001.npz")) as data:
                self.assertEqual(set(data["semantic_labels"]), {-1, 10})
                self.assertEqual(int(data["reviewed_mask"].sum()), 2)
                self.assertTrue(np.all(data["instance_ids"][data["semantic_labels"] == -1] == -1))
            metadata = json.loads(restarted.export_file("revision-000001.json").read_text())
            self.assertEqual(metadata["taxonomy_catalog_id"], registry.read()["catalog_id"])
            self.assertEqual(metadata["taxonomy"][-1], category)

    def test_category_growth_never_rewrites_annotations_or_prior_exports(self):
        registry = CategoryRegistry(Path(self.temp.name) / "categories.json")
        store = AnnotationStore(self.directory, registry=registry)
        first_export = store.export(0)
        before_annotations = store.annotations_path.read_bytes()
        before_files = {ext: store.export_file("revision-000000." + ext).read_bytes() for ext in ("npz", "json")}
        category = registry.create("Archivador")["category"]
        self.assertEqual(store.read()["revision"], 0)
        self.assertEqual(store.annotations_path.read_bytes(), before_annotations)
        self.assertEqual(store.describe()["taxonomy"][-1], category)
        self.assertEqual(store.export(0), first_export)
        for ext, content in before_files.items():
            self.assertEqual(store.export_file("revision-000000." + ext).read_bytes(), content)
        store.update({"revision": 0, "action": "label", "label": category["key"], "group_ids": ["floor_a"]})
        store.export(1)
        registry.create("Ventilador")
        after_label = store.annotations_path.read_bytes()
        store.describe(); store.export(1)
        self.assertEqual(store.annotations_path.read_bytes(), after_label)

    def test_custom_annotations_reject_missing_foreign_and_remapped_catalogs(self):
        registry_path = Path(self.temp.name) / "categories.json"
        registry = CategoryRegistry(registry_path)
        category = registry.create("Estante")["category"]
        store = AnnotationStore(self.directory, registry=registry)
        store.update({"revision": 0, "action": "label", "label": category["key"], "group_ids": ["object_a"]})
        with self.assertRaises(DatasetChanged):
            AnnotationStore(self.directory)
        foreign = CategoryRegistry(Path(self.temp.name) / "other-categories.json")
        foreign.create("Estante")
        with self.assertRaises(DatasetChanged):
            AnnotationStore(self.directory, registry=foreign)
        # Even replacing both registry files cannot bypass the persisted dataset prefix.
        saved = json.loads(registry_path.read_text())
        saved["categories"][-1]["name"] = "Televisor"
        registry_path.write_text(json.dumps(saved))
        registry.identity_path.unlink()
        remapped = CategoryRegistry(registry_path)
        with self.assertRaises(DatasetChanged):
            AnnotationStore(self.directory, registry=remapped)
        registry_path.unlink()
        with self.assertRaises(CategoryRegistryError):
            CategoryRegistry(registry_path)


if __name__ == "__main__":
    unittest.main()
