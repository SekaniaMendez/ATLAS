"""Compact parametric description; mesh buffers live in the GLB, not here."""
import json
import math
import numpy as np


def json_value(value):
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return round(float(value), 8)
    return value


def write_json(path, value):
    path.write_text(json.dumps(json_value(value), ensure_ascii=False, allow_nan=False,
                               separators=(",", ":")) + "\n", encoding="utf-8")


def scene_description(patches, metadata, structural_frame):
    surfaces = []
    for patch in patches:
        axes, origin = np.asarray(patch["axes"]), np.asarray(patch["origin"])
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = axes, origin
        vertex_bounds = np.asarray(patch["vertices"])
        surfaces.append({
            "id": patch["id"], "geometry": "triangle_mesh" if patch["kind"] == "freeform" else "bounded_plane",
            "orientation": patch["kind"], "semantic_label": None,
            "frame": {"parent": "slam_map", "T_parent_local_rows": transform,
                      "origin_definition": patch.get("origin_definition", "mesh_centroid")},
            "plane": None if patch["kind"] == "freeform" else {
                "normal_in_map": patch["normal"], "offset_m": patch["offset"],
                "equation": "dot(normal, point_in_map) + offset_m = 0"},
            "area_m2": patch["area_m2"], "fit_rmse_m": patch["rmse_m"],
            "support_points": len(patch["support_indices"]), "holes": patch.get("hole_count", 0),
            "local_bounds_m": [vertex_bounds.min(axis=0), vertex_bounds.max(axis=0)],
            "vertices": len(patch["vertices"]), "triangles": len(patch["triangles"]),
            "mesh_object": patch["id"], "measured_surface_hypothesis": True,
            "reconstruction": {key: patch[key] for key in (
                "surface_model", "support_max_connection_m", "support_max_circumradius_m",
                "mesh_to_measurement_bound_m", "simplification") if key in patch},
        })
    return {
        "schema_version": 1, "generator": "ATLAS structural geometry prototype",
        "units": "metres", "map_frame": "slam_map", "map_up_axis": "+Z",
        "georeferenced": False, "semantic_recognition": False, "closed_volume": False,
        "snapshot_ids_only": True, "unobserved_space": "unknown",
        "glb_axis_conversion": "Root Rx(-90 degrees) maps ATLAS Z-up to glTF Y-up; source map unchanged.",
        "mesh_file": "scene.glb", "structural_frame": structural_frame,
        "source": metadata, "surfaces": surfaces,
    }


def retention_decision(quality, metadata, ratio):
    """Keep the temporary measurement backup unless all declared gates pass."""
    session = metadata.get("slam_session", {})
    reasons = []
    coverage = quality.get("point_coverage")
    unsupported = quality.get("unsupported_area_fraction")
    if (not all(isinstance(value, (int, float)) and math.isfinite(value)
                for value in (coverage, unsupported, ratio))
            or not 0 <= coverage <= 1 or not 0 <= unsupported <= 1 or ratio <= 0):
        return False, ["Las métricas de validación son inválidas; se conservan las mediciones."]
    if quality["point_coverage"] < .99:
        reasons.append("Más del 1 % de las mediciones queda fuera de la tolerancia geométrica.")
    if quality["unsupported_area_fraction"] > .01:
        reasons.append("Más del 1 % del área muestreada carece de mediciones próximas.")
    if ratio < 2.0:
        reasons.append("La reducción del modelo es menor que 2×.")
    if session.get("tracking_state") != "2" or session.get("fatal_error") != "0":
        reasons.append("La sesión no declara seguimiento final válido sin error fatal.")
    if not metadata.get("gravity_aligned", False):
        reasons.append("La orientación respecto a la gravedad no está documentada por el SLAM.")
    return not reasons, reasons
