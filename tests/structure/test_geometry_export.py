"""Regression checks for surface-only GLB frames, layout and safe HTML export."""
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "structure"))
from export_geometry import export_glb, write_viewer


def patch():
    return {
        "id": "plane_001", "kind": "vertical", "origin": [2.0, -3.0, 1.0],
        "axes": [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
        "normal": [0, -1, 0], "offset": -3.0, "rmse_m": 0.006,
        "area_m2": 4.0, "support_indices": np.arange(100),
        # An unused point must not sneak into the exported geometry.
        "vertices": [[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0], [80, 90, 100]],
        "triangles": np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64),
    }


def read_glb(path):
    raw = path.read_bytes()
    magic, version, total = struct.unpack_from("<4sII", raw)
    if (magic, version, total) != (b"glTF", 2, len(raw)):
        raise AssertionError("Invalid GLB header")
    json_length, kind = struct.unpack_from("<I4s", raw, 12)
    if kind != b"JSON" or json_length % 4:
        raise AssertionError("Invalid JSON chunk")
    description = json.loads(raw[20:20 + json_length])
    binary_length, kind = struct.unpack_from("<I4s", raw, 20 + json_length)
    if kind != b"BIN\0" or binary_length % 4 or total != 28 + json_length + binary_length:
        raise AssertionError("Invalid binary chunk")
    return description, raw[28 + json_length:]


def read_accessor(doc, binary, index):
    accessor = doc["accessors"][index]
    view = doc["bufferViews"][accessor["bufferView"]]
    dtype = {5123: "<u2", 5125: "<u4", 5126: "<f4"}[accessor["componentType"]]
    width = {"SCALAR": 1, "VEC3": 3}[accessor["type"]]
    offset = view["byteOffset"] + accessor.get("byteOffset", 0)
    assert offset % 4 == 0
    assert offset + accessor["count"] * width * np.dtype(dtype).itemsize <= len(binary)
    return np.frombuffer(binary, dtype=dtype, count=accessor["count"] * width,
                         offset=offset).reshape(-1, width)


class GeometryExportTest(unittest.TestCase):
    def test_glb_preserves_frame_scale_topology_and_excludes_unused_points(self):
        source = patch()
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "scene.glb"
            stats = export_glb(target, [source], {"source_name": "capture.ply"})
            doc, binary = read_glb(target)
            self.assertEqual(stats["bytes"], target.stat().st_size)
            self.assertEqual((stats["objects"], stats["vertices"], stats["triangles"]), (1, 4, 2))
            self.assertEqual(doc["scenes"][0]["nodes"], [0])
            self.assertEqual(doc["nodes"][0]["children"], [1])
            self.assertTrue(np.allclose(doc["nodes"][0]["rotation"], [-math.sqrt(.5), 0, 0, math.sqrt(.5)]))
            node = doc["nodes"][1]
            transform = np.asarray(node["matrix"]).reshape(4, 4, order="F")
            self.assertTrue(np.array_equal(transform[:3, :3], source["axes"]))
            self.assertTrue(np.array_equal(transform[:3, 3], source["origin"]))
            primitive = doc["meshes"][0]["primitives"][0]
            self.assertEqual(primitive["mode"], 4)
            positions = read_accessor(doc, binary, primitive["attributes"]["POSITION"])
            triangles = read_accessor(doc, binary, primitive["indices"]).reshape(-1, 3)
            normals = read_accessor(doc, binary, primitive["attributes"]["NORMAL"])
            self.assertTrue(np.array_equal(triangles, source["triangles"]))
            self.assertTrue(np.allclose(np.linalg.norm(normals, axis=1), 1))
            world = np.einsum("ij,nj->ni", transform[:3, :3], positions) + transform[:3, 3]
            # 2x2 metre vertical patch at map Y=-3, from map Z=0 through Z=2.
            self.assertTrue(np.allclose(world.min(axis=0), [1, -3, 0]))
            self.assertTrue(np.allclose(world.max(axis=0), [3, -3, 2]))
            # Root conversion: map (X,Y,Z) becomes glTF (X,Z,-Y).
            gltf = world[:, [0, 2, 1]] * [1, 1, -1]
            self.assertTrue(np.allclose(gltf.min(axis=0), [1, 0, 3]))
            self.assertNotIn("support_indices", node["extras"])
            self.assertEqual(node["extras"]["support_count"], 100)

    def test_index_width_is_lossless_and_buffer_views_remain_aligned(self):
        # One uint16 triangle leaves two padding bytes before the next mesh.
        small = dict(patch(), triangles=np.array([[0, 1, 2]], dtype=np.int64))
        group_count = 21846
        vertices = np.tile([[0, 0, 0], [1, 0, 0], [0, 1, 0]], (group_count, 1))
        vertices[:, 0] += np.repeat(np.arange(group_count) * 2, 3)
        large = dict(patch(), id="large", vertices=vertices,
                     triangles=np.arange(len(vertices)).reshape(-1, 3))
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "mixed.glb"
            export_glb(target, [small, large])
            doc, binary = read_glb(target)
            for mesh, expected_type, expected_count in zip(doc["meshes"], [5123, 5125], [3, 65538]):
                index = mesh["primitives"][0]["indices"]
                self.assertEqual(doc["accessors"][index]["componentType"], expected_type)
                values = read_accessor(doc, binary, index).ravel()
                self.assertTrue(np.array_equal(values, np.arange(expected_count)))
            self.assertTrue(all(view["byteOffset"] % 4 == 0 for view in doc["bufferViews"]))

    def test_invalid_frames_indices_and_empty_models_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "bad.glb"
            for replacement in (
                {"axes": np.diag([1, 1, -1])}, {"origin": [0, 0, float("nan")]},
                {"triangles": [[0, 1, 80]]}, {"triangles": [[0.0, 1.0, 2.1]]},
                {"triangles": [[0, 0, 1]]}, {"triangles": []},
            ):
                with self.subTest(replacement=replacement), self.assertRaises(ValueError):
                    export_glb(target, [dict(patch(), **replacement)])
            with self.assertRaises(ValueError):
                export_glb(target, [])

    def test_html_is_standalone_escaped_and_preserves_unevaluated_metrics(self):
        source = dict(patch(), kind="freeform", rmse_m=None, offset=None)
        hostile = "</script><script>throw Error('unexpected')</script>"
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "viewer.html"
            write_viewer(target, [source], {"source_name": hostile, "model_bytes": 1000},
                         {"origin_m": [0, 0, 0], "axes": np.eye(3)})
            html = target.read_text()
            self.assertNotIn(hostile, html)
            self.assertNotIn("__ATLAS_SCRIPT__", html)
            self.assertNotIn("__ATLAS_DATA__", html)
            self.assertNotIn("<script src=", html)
            embedded = html.split('<script type="application/json" id="atlas-data">', 1)[1].split("</script>", 1)[0]
            data = json.loads(embedded)
            self.assertEqual(data["metadata"]["source_name"], hostile)
            self.assertIsNone(data["surfaces"][0]["rmse_m"])
            self.assertEqual(data["structural_frame"]["origin_m"], [0, 0, 0])
            self.assertEqual(len(data["surfaces"][0]["vertices"]), 4)


if __name__ == "__main__":
    unittest.main()
