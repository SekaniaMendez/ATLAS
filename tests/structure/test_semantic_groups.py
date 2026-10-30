"""Meaningful safeguards for geometric suggestions and training-point ownership."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "structure"))
from semantic_groups import SemanticGroupConfig, build_groups


def plane_cloud(origin, u, v, width, height, step=.08):
    a, b = np.meshgrid(np.arange(0, width + step / 2, step),
                       np.arange(0, height + step / 2, step))
    return np.asarray(origin) + a.ravel()[:, None] * u + b.ravel()[:, None] * v


class Scene:
    def __init__(self):
        self.points, self.patches, self.count = [], [], 0

    def plane(self, name, origin, u, v, width, height, step=.08):
        u, v = np.asarray(u, float), np.asarray(v, float)
        cloud = plane_cloud(origin, u, v, width, height, step)
        indices = np.arange(self.count, self.count + len(cloud))
        normal = np.cross(u, v)
        self.patches.append({"id": name, "support_indices": indices, "normal": normal,
                             "origin": np.asarray(origin, float), "area_m2": width * height,
                             "kind": "horizontal" if abs(normal[2]) > .9 else "vertical"})
        self.points.append(cloud)
        self.count += len(cloud)
        return indices

    def extra(self, points):
        points = np.asarray(points, float).reshape(-1, 3)
        indices = np.arange(self.count, self.count + len(points))
        self.points.append(points)
        self.count += len(points)
        return indices

    def cloud(self):
        return np.concatenate(self.points)


def room(ceiling=True, floor=True):
    scene = Scene()
    if floor:
        scene.plane("floor", [0, 0, 0], [1, 0, 0], [0, 1, 0], 3, 3)
    scene.plane("wall_x", [0, 0, 0], [0, 1, 0], [0, 0, 1], 3, 3)
    scene.plane("wall_y", [0, 0, 0], [1, 0, 0], [0, 0, 1], 3, 3)
    if ceiling:
        scene.plane("ceiling", [0, 0, 3], [1, 0, 0], [0, 1, 0], 3, 3)
    return scene


class SemanticGroupsTest(unittest.TestCase):
    def assert_partition(self, points, result):
        assignment = result["point_group_ids"]
        self.assertEqual(assignment.dtype, np.int32)
        self.assertEqual(len(assignment), len(points))
        self.assertTrue(np.all(assignment >= 0))
        self.assertEqual(sum(group["point_count"] for group in result["groups"]), len(points))
        self.assertEqual(len(set(group["id"] for group in result["groups"])), len(result["groups"]))
        for index, group in enumerate(result["groups"]):
            self.assertEqual(np.count_nonzero(assignment == index), group["point_count"])

    def test_room_suggestions_do_not_label_table_or_small_vertical_furniture(self):
        scene = room()
        table = scene.plane("tabletop", [1, 1, .75], [1, 0, 0], [0, 1, 0], 1, 1)
        cabinet = scene.plane("cabinet_front", [2.5, 1, .1], [0, 1, 0], [0, 0, 1], .6, 1)
        points = scene.cloud()
        original = points.copy()
        result = build_groups(points, scene.patches, gravity_aligned=True)
        self.assert_partition(points, result)
        self.assertTrue(np.array_equal(original, points))
        labels = {group["suggested_label"] for group in result["groups"]}
        self.assertTrue({"floor", "wall", "ceiling"} <= labels)
        for indices in (table, cabinet):
            for group_index in np.unique(result["point_group_ids"][indices]):
                group = result["groups"][group_index]
                self.assertIsNone(group["suggested_label"])
                self.assertEqual(group["kind"], "object")
        self.assertFalse(result["diagnostics"]["learned_model"])
        self.assertTrue(result["diagnostics"]["labels_require_human_review"])

    def test_absent_ceiling_is_not_invented(self):
        scene = room(ceiling=False)
        scene.plane("tabletop", [1, 1, .75], [1, 0, 0], [0, 1, 0], 1, 1)
        result = build_groups(scene.cloud(), scene.patches, gravity_aligned=True)
        self.assertNotIn("ceiling", [group["suggested_label"] for group in result["groups"]])
        self.assertIn("floor", [group["suggested_label"] for group in result["groups"]])

    def test_absent_floor_is_not_invented_and_ceiling_can_use_wall_context(self):
        scene = room(floor=False)
        scene.plane("tabletop", [1, 1, .75], [1, 0, 0], [0, 1, 0], 1, 1)
        result = build_groups(scene.cloud(), scene.patches, gravity_aligned=True)
        self.assertNotIn("floor", [group["suggested_label"] for group in result["groups"]])
        self.assertIn("ceiling", [group["suggested_label"] for group in result["groups"]])
        self.assertIsNone(result["diagnostics"]["floor_level_m"])

    def test_gravity_is_required_and_single_table_is_not_a_room(self):
        scene = room()
        result = build_groups(scene.cloud(), scene.patches, gravity_aligned=False)
        self.assertTrue(all(group["suggested_label"] is None for group in result["groups"]))
        table = Scene()
        table.plane("large_table", [0, 0, .75], [1, 0, 0], [0, 1, 0], 2, 2)
        result = build_groups(table.cloud(), table.patches, gravity_aligned=True)
        self.assertTrue(all(group["suggested_label"] is None for group in result["groups"]))

    def test_support_is_bounded_not_an_infinite_plane(self):
        scene = room()
        remote = scene.extra(plane_cloud([8, 8, 0], np.array([1, 0, 0]), np.array([0, 1, 0]), 1, 1))
        points = scene.cloud()
        result = build_groups(points, scene.patches, gravity_aligned=True)
        self.assert_partition(points, result)
        for index in np.unique(result["point_group_ids"][remote]):
            self.assertIsNone(result["groups"][index]["suggested_label"])

    def test_sparse_bridge_does_not_merge_dense_objects_and_noise_is_retained(self):
        scene = Scene()
        first = scene.extra(plane_cloud([0, 0, .6], np.array([1, 0, 0]), np.array([0, 1, 0]), .4, .4, .05))
        second = scene.extra(plane_cloud([1.2, 0, .6], np.array([1, 0, 0]), np.array([0, 1, 0]), .4, .4, .05))
        scene.extra([[x, .2, .6] for x in np.arange(.55, 1.1, .15)])
        noise = scene.extra([[8, 8, 8], [10, 10, 10]])
        points = scene.cloud()
        result = build_groups(points, [], gravity_aligned=True)
        self.assert_partition(points, result)
        self.assertEqual(result["diagnostics"]["object_groups"], 2)
        self.assertNotEqual(result["point_group_ids"][first[20]], result["point_group_ids"][second[20]])
        for index in np.unique(result["point_group_ids"][noise]):
            self.assertEqual(result["groups"][index]["kind"], "unassigned")

    def test_point_permutation_preserves_ids_memberships_and_suggestions(self):
        scene = room()
        scene.plane("tabletop", [1, 1, .75], [1, 0, 0], [0, 1, 0], 1, 1)
        points = scene.cloud()
        first = build_groups(points, scene.patches, gravity_aligned=True)
        order = np.random.default_rng(3).permutation(len(points))
        inverse = np.argsort(order)
        patches = [{**patch, "support_indices": inverse[patch["support_indices"]]}
                   for patch in reversed(scene.patches)]
        second = build_groups(points[order], patches, gravity_aligned=True)
        self.assertEqual(first["groups"], second["groups"])
        self.assertTrue(np.array_equal(first["point_group_ids"], second["point_group_ids"][inverse]))

    def test_limits_abstain_without_discarding_points_and_validate_input(self):
        cloud = plane_cloud([0, 0, .6], np.array([1, 0, 0]), np.array([0, 1, 0]), 1, 1)
        result = build_groups(cloud, [], config={"max_grouping_voxels": 10})
        self.assert_partition(cloud, result)
        self.assertTrue(result["diagnostics"]["grouping_budget_exceeded"])
        self.assertEqual(result["groups"][0]["kind"], "unassigned")
        self.assertEqual(build_groups(np.empty((0, 3)), [])["groups"], [])
        for invalid in (np.zeros((2, 2)), np.array([[0, 0, np.nan]])):
            with self.assertRaises(ValueError):
                build_groups(invalid, [])
        with self.assertRaises(ValueError):
            build_groups(cloud, [], config=replace(SemanticGroupConfig(), object_radius_m=-1))
        with self.assertRaises(ValueError):
            build_groups(cloud, [], config={"max_input_points": 2})


if __name__ == "__main__":
    unittest.main()
