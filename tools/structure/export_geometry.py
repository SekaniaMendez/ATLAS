"""Compact glTF/HTML exports of derived surfaces, never raw sensor point clouds.

Patch vertices are local. ``axes`` contains local X/Y/Z as columns in the
ATLAS map (metres, Z up). glTF nodes retain those transforms below one root
rotation of -90 degrees around X, which converts ATLAS Z up to glTF Y up.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import struct

import numpy as np


_PALETTE = (
    (0.13, 0.64, 0.75), (0.96, 0.60, 0.27), (0.54, 0.47, 0.82),
    (0.33, 0.71, 0.52), (0.90, 0.39, 0.46), (0.35, 0.55, 0.88),
    (0.80, 0.66, 0.26), (0.72, 0.43, 0.68), (0.28, 0.71, 0.72),
)


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(v) for v in value]
    return value


def _mesh(patch):
    vertices = np.asarray(patch["vertices"], dtype=np.float64).reshape(-1, 3)
    raw_triangles = np.asarray(patch["triangles"])
    if raw_triangles.size == 0:
        triangles = np.empty((0, 3), dtype=np.int64)
    else:
        if not np.issubdtype(raw_triangles.dtype, np.integer):
            raise ValueError("Triangle indices must be integers")
        triangles = raw_triangles.astype(np.int64, copy=False).reshape(-1, 3)
    origin = np.asarray(patch.get("origin", [0, 0, 0]), dtype=np.float64)
    axes = np.asarray(patch.get("axes", np.eye(3)), dtype=np.float64)
    if origin.shape != (3,) or axes.shape != (3, 3):
        raise ValueError("A patch needs a 3-vector origin and a 3x3 axes matrix")
    if not all(np.isfinite(a).all() for a in (vertices, origin, axes)):
        raise ValueError("Non-finite surface geometry")
    if not np.allclose(np.einsum("ki,kj->ij", axes, axes), np.eye(3), atol=1e-6) or not np.isclose(
            np.linalg.det(axes), 1.0, atol=1e-6):
        raise ValueError("Patch axes must be an orthonormal right-handed basis")
    if len(triangles) and (triangles.min() < 0 or triangles.max() >= len(vertices)):
        raise ValueError("Surface triangle index is outside its vertex array")
    if not len(triangles):
        return None
    # Unused mesh vertices do not represent surfaces; do not export them as points.
    used, indices = np.unique(triangles, return_inverse=True)
    vertices, triangles = vertices[used], indices.reshape(-1, 3)
    normals = np.cross(vertices[triangles[:, 1]] - vertices[triangles[:, 0]],
                       vertices[triangles[:, 2]] - vertices[triangles[:, 0]])
    if np.any(np.linalg.norm(normals, axis=1) <= 1e-12):
        raise ValueError("Degenerate triangle in surface export")
    return vertices, triangles, origin, axes


def _color(index, patch):
    return (0.55, 0.62, 0.66) if patch.get("kind") == "freeform" else _PALETTE[index % len(_PALETTE)]


def _patch_info(patch, index):
    info = {
        "id": str(patch.get("id", f"surface_{index + 1:03d}")),
        "kind": str(patch.get("kind", "freeform")),
        "color": list(_color(index, patch)),
        "area_m2": float(patch.get("area_m2", 0.0)),
        "rmse_m": (float(patch["rmse_m"]) if patch.get("rmse_m") is not None else None),
        "support_count": int(patch.get("support_count", len(patch.get("support_indices", [])))),
    }
    for key in ("normal", "offset", "classification", "label", "confidence"):
        if key in patch:
            info[key] = _json_value(patch[key])
    return info


def export_glb(path, patches, metadata=None):
    """Write a glTF 2.0 binary scene; return object/vertex/triangle/byte counts.

    Empty patches are omitted. Invalid transforms, indices and degenerate faces
    fail explicitly. Materials are double-sided because these are observed open
    surfaces, not necessarily the boundary of a solid object.
    """
    path = Path(path)
    blob = bytearray()
    doc = {
        "asset": {"version": "2.0", "generator": "ATLAS structural geometry"},
        "scene": 0, "scenes": [{"nodes": [0]}],
        "nodes": [{"name": "ATLAS map (metres; Z-up to glTF Y-up)",
                   "rotation": [-math.sqrt(0.5), 0, 0, math.sqrt(0.5)], "children": []}],
        "meshes": [], "materials": [], "bufferViews": [], "accessors": [],
        "extras": {"source_frame": "ATLAS local map", "source_up_axis": "Z",
                   "units": "metres", "surface_model": "observed, open surfaces",
                   "metadata": _json_value(metadata or {})},
    }
    counts = {"objects": 0, "vertices": 0, "triangles": 0}

    def accessor(array, component, kind, target, bounds=False):
        while len(blob) % 4:
            blob.append(0)
        payload = array.tobytes(order="C")
        view = len(doc["bufferViews"])
        doc["bufferViews"].append({"buffer": 0, "byteOffset": len(blob),
                                   "byteLength": len(payload), "target": target})
        blob.extend(payload)
        entry = {"bufferView": view, "componentType": component,
                 "count": len(array), "type": kind}
        if bounds:
            entry.update(min=array.min(axis=0).tolist(), max=array.max(axis=0).tolist())
        doc["accessors"].append(entry)
        return len(doc["accessors"]) - 1

    for index, patch in enumerate(patches):
        surface = _mesh(patch)
        if surface is None:
            continue
        vertices, triangles, origin, axes = surface
        info = _patch_info(patch, index)
        positions = vertices.astype("<f4")
        if not np.isfinite(positions).all():
            raise ValueError("Surface positions exceed float32 range")
        normals = np.zeros_like(vertices)
        faces = np.cross(vertices[triangles[:, 1]] - vertices[triangles[:, 0]],
                         vertices[triangles[:, 2]] - vertices[triangles[:, 0]])
        for corner in range(3):
            np.add.at(normals, triangles[:, corner], faces)
        lengths = np.linalg.norm(normals, axis=1)
        safe = lengths > 1e-12
        normals[safe] /= lengths[safe, None]
        normals[~safe] = [0, 0, 1]
        pos_accessor = accessor(positions, 5126, "VEC3", 34962, True)
        normal_accessor = accessor(normals.astype("<f4"), 5126, "VEC3", 34962)
        compact_indices = int(triangles.max()) < 65536
        idx_accessor = accessor(triangles.astype("<u2" if compact_indices else "<u4").reshape(-1),
                                5123 if compact_indices else 5125, "SCALAR", 34963)
        material = len(doc["materials"])
        doc["materials"].append({"name": info["id"], "doubleSided": True,
            "pbrMetallicRoughness": {"baseColorFactor": [*info["color"], 1.0],
                                    "metallicFactor": 0.0, "roughnessFactor": 0.85}})
        mesh_index = len(doc["meshes"])
        doc["meshes"].append({"name": info["id"], "primitives": [{
            "attributes": {"POSITION": pos_accessor, "NORMAL": normal_accessor},
            "indices": idx_accessor, "material": material, "mode": 4}]})
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = axes, origin
        node_index = len(doc["nodes"])
        doc["nodes"].append({"name": info["id"], "mesh": mesh_index,
                             "matrix": transform.flatten(order="F").tolist(), "extras": info})
        doc["nodes"][0]["children"].append(node_index)
        counts["objects"] += 1
        counts["vertices"] += len(vertices)
        counts["triangles"] += len(triangles)
    if not counts["objects"]:
        raise ValueError("No triangles to export; a point cloud is not a surface model")
    doc["buffers"] = [{"byteLength": len(blob)}]
    content = json.dumps(doc, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
    content += b" " * (-len(content) % 4)
    blob.extend(b"\0" * (-len(blob) % 4))
    total = 12 + 8 + len(content) + 8 + len(blob)
    if total >= 2**32:
        raise ValueError("GLB exceeds its 32-bit container limit")
    with path.open("wb") as stream:
        stream.write(struct.pack("<4sII", b"glTF", 2, total))
        stream.write(struct.pack("<I4s", len(content), b"JSON"))
        stream.write(content)
        stream.write(struct.pack("<I4s", len(blob), b"BIN\0"))
        stream.write(blob)
    return {**counts, "bytes": total}


def write_viewer(path, patches, metadata=None, structural_frame=None):
    """Write one standalone interactive HTML file; all geometry is embedded.

    Metadata may include ``source_name``, ``input_bytes``, ``model_bytes`` (GLB
    plus references) or ``geometry_bytes`` (GLB only),
    ``point_coverage`` (0..1), ``coverage_tolerance_m``, ``model_filename`` and
    ``warnings``. Missing metrics remain unavailable rather than guessed.
    """
    surfaces = []
    for index, patch in enumerate(patches):
        mesh = _mesh(patch)
        if mesh is None:
            continue
        vertices, triangles, origin, axes = mesh
        surfaces.append({**_patch_info(patch, index),
                         "origin": origin.tolist(), "axes": axes.tolist(),
                         "vertices": np.round(vertices, 7).tolist(),
                         "triangles": triangles.tolist()})
    data = {"surfaces": surfaces, "metadata": _json_value(metadata or {}),
            "structural_frame": _json_value(structural_frame)}
    # An untrusted source filename or label must never terminate the JSON script.
    encoded = json.dumps(data, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    for char, replacement in (("&", "\\u0026"), ("<", "\\u003c"), (">", "\\u003e"),
                              ("\u2028", "\\u2028"), ("\u2029", "\\u2029")):
        encoded = encoded.replace(char, replacement)
    template = Path(__file__).with_name("viewer.html").read_text(encoding="utf-8")
    if template.count("__ATLAS_DATA__") != 1 or template.count("__ATLAS_SCRIPT__") != 1:
        raise ValueError("The viewer template must contain one data and one script placeholder")
    scripts = "\n".join(Path(__file__).with_name(name).read_text(encoding="utf-8")
                        for name in ("viewer_scene.js", "viewer_runtime.js"))
    template = template.replace("__ATLAS_SCRIPT__", scripts)
    path = Path(path)
    path.write_text(template.replace("__ATLAS_DATA__", encoded), encoding="utf-8")
    return {"objects": len(surfaces), "bytes": path.stat().st_size}
