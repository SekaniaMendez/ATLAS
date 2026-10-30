"""Conservative structural suggestions and reviewable point-cloud groups.

This is geometric bootstrapping, not a trained object recognizer. Every input
point is retained and receives exactly one group. Labels are suggestions only;
human confirmation belongs to the annotation layer. Coordinates remain in the
input, metric, gravity-aligned SLAM frame. IDs describe this exact snapshot.
"""
from dataclasses import asdict, dataclass
import hashlib
import math

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class SemanticGroupConfig:
    orientation_angle_degrees: float = 12.0
    structure_distance_m: float = 0.04
    structure_support_radius_m: float = 0.18
    floor_min_area_m2: float = 0.8
    ceiling_min_area_m2: float = 0.6
    wall_min_area_m2: float = 0.6
    wall_min_height_m: float = 1.4
    wall_min_width_m: float = 0.55
    level_tolerance_m: float = 0.3
    min_room_height_m: float = 1.8
    object_voxel_m: float = 0.06
    object_radius_m: float = 0.18
    object_min_neighbors: int = 5
    object_max_neighbors: int = 48
    object_min_points: int = 35
    object_min_voxels: int = 8
    max_grouping_voxels: int = 300_000
    max_input_points: int = 2_000_000
    max_object_groups: int = 512

    def validate(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
            if name.startswith("max_") or name in {
                    "object_min_neighbors", "object_max_neighbors", "object_min_points", "object_min_voxels"}:
                if type(value) is not int:
                    raise ValueError(f"{name} must be an integer")
        if not 0 < self.orientation_angle_degrees < 45:
            raise ValueError("orientation_angle_degrees must be between 0 and 45")
        if self.object_min_neighbors < 2 or self.object_max_neighbors < self.object_min_neighbors:
            raise ValueError("neighbor limits must satisfy 2 <= minimum <= maximum")
        if self.object_radius_m < self.object_voxel_m * 1.5:
            raise ValueError("object_radius_m must be at least 1.5 times object_voxel_m")
        return self


def _patches(points, patches):
    result = []
    for patch in patches:
        if patch.get("kind") == "freeform":
            continue
        raw = np.asarray(patch.get("support_indices", []))
        if raw.size == 0:
            continue
        if raw.ndim != 1 or not np.issubdtype(raw.dtype, np.integer):
            raise ValueError("plane support_indices must be a one-dimensional integer array")
        indices = np.unique(raw.astype(np.int64))
        if indices.min() < 0 or indices.max() >= len(points):
            raise ValueError("plane support index is outside points")
        normal, origin = np.asarray(patch["normal"], float), np.asarray(patch["origin"], float)
        area = float(patch["area_m2"])
        if (normal.shape != (3,) or origin.shape != (3,) or not np.isfinite(normal).all()
                or not np.isfinite(origin).all() or not math.isfinite(area) or area < 0
                or np.linalg.norm(normal) < 1e-8):
            raise ValueError("invalid plane geometry")
        normal = normal / np.linalg.norm(normal)
        support = points[indices]
        lo, hi = np.quantile(support, [0.01, 0.99], axis=0)
        result.append({"id": str(patch["id"]), "indices": indices, "normal": normal,
                       "origin": origin, "area": area, "lo": lo, "hi": hi,
                       "level": float(np.median(support[:, 2])),
                       "height": float(hi[2] - lo[2]),
                       "width": float(np.linalg.norm(hi[:2] - lo[:2]))})
    # Tie-breaking must depend on geometry, not the order of extracted planes.
    return sorted(result, key=lambda p: (-p["area"], tuple(p["origin"]), p["id"]))


def _structural_suggestions(points, patches, gravity_aligned, config):
    if not gravity_aligned or not len(points):
        return [], {"floor_level_m": None, "ceiling_levels_m": [],
                    "abstention": "Sin gravedad documentada: no se proponen suelo, paredes ni techo."}
    low_z, high_z = np.quantile(points[:, 2], [0.05, 0.95])
    horizontal_cos = math.cos(math.radians(config.orientation_angle_degrees))
    vertical_cos = math.sin(math.radians(config.orientation_angle_degrees))
    walls = [p for p in patches if abs(p["normal"][2]) <= vertical_cos
             and p["height"] >= config.wall_min_height_m
             and p["width"] >= config.wall_min_width_m
             and p["area"] >= config.wall_min_area_m2]
    horizontal = [p for p in patches if abs(p["normal"][2]) >= horizontal_cos]
    floors = [p for p in horizontal if p["area"] >= config.floor_min_area_m2
              and p["level"] <= low_z + config.level_tolerance_m
              and high_z - p["level"] >= config.min_room_height_m]
    # A floor hypothesis needs vertical context, not just a low tabletop.
    floors = [p for p in floors if any(
        w["hi"][2] - p["level"] >= config.wall_min_height_m
        and w["lo"][2] <= p["level"] + config.level_tolerance_m for w in walls)]
    floor_level = min((p["level"] for p in floors), default=None)
    floors = [p for p in floors if p["level"] <= floor_level + config.level_tolerance_m]
    ceiling_base = floor_level if floor_level is not None else low_z
    ceilings = [p for p in horizontal if p["area"] >= config.ceiling_min_area_m2
                and p["level"] >= high_z - config.level_tolerance_m
                and p["level"] - ceiling_base >= config.min_room_height_m
                and any(abs(w["hi"][2] - p["level"]) <= config.level_tolerance_m
                        and p["level"] - w["lo"][2] >= config.wall_min_height_m for w in walls)]
    suggestions = []
    for label, selected, reason in (
        ("floor", floors, "Plano horizontal amplio en el nivel inferior, con soporte vertical por encima."),
        ("ceiling", ceilings, "Plano horizontal alto y amplio, coherente con la parte superior de paredes observadas."),
        ("wall", walls, "Plano vertical con altura y extensión de pared; requiere revisión, puede incluir puertas o muebles altos."),
    ):
        for patch in selected:
            confidence = {"floor": .82, "ceiling": .80, "wall": .72}[label]
            suggestions.append((patch, label, confidence, reason))
    diagnostics = {"floor_level_m": floor_level,
                   "ceiling_levels_m": sorted(set(round(p["level"], 4) for p in ceilings)),
                   "observed_height_quantiles_m": [float(low_z), float(high_z)],
                   "abstention": None if floors else "No hay soporte suficiente para proponer un suelo; no se inventa."}
    return suggestions, diagnostics


def _descriptor(points, indices, kind, label=None, confidence=0.0, reason="", surface_ids=()):
    cloud = points[indices]
    order = np.lexsort((cloud[:, 2], cloud[:, 1], cloud[:, 0]))
    digest = hashlib.sha256(np.asarray(cloud[order], dtype="<f8").tobytes()).hexdigest()[:20]
    minimum, maximum = cloud.min(axis=0), cloud.max(axis=0)
    return {"id": "group_" + digest, "kind": kind, "suggested_label": label,
            "confidence": confidence, "reason": reason, "point_count": len(indices),
            "bounds_m": [minimum.tolist(), maximum.tolist()],
            "centroid_m": cloud[order].mean(axis=0).tolist(),
            "dimensions_m": (maximum - minimum).tolist(), "surface_ids": sorted(surface_ids)}


def _object_components(points, indices, config):
    """Bounded-neighbor DBSCAN variant: sparse border points cannot join cores.

    One real point per voxel, up to object_max_neighbors links per voxel. This
    bounds the graph storage rather than materializing every radius pair.
    Small components and sparse observations remain available as unassigned.
    """
    if not len(indices):
        return [], indices, {"grouping_voxels": 0, "grouping_budget_exceeded": False}
    remaining = points[indices]
    cells = np.floor(remaining / config.object_voxel_m)
    if np.abs(cells).max() > np.iinfo(np.int64).max / 2:
        raise ValueError("point coordinates exceed the grouping grid range")
    cells = cells.astype(np.int64)
    _, first, inverse = np.unique(cells, axis=0, return_index=True, return_inverse=True)
    # Use the lexicographically first real point in each cell, independent of
    # input order; cell centroids would create synthetic training samples.
    sort = np.lexsort((remaining[:, 2], remaining[:, 1], remaining[:, 0], inverse))
    sorted_cell = inverse[sort]
    first = sort[np.r_[True, sorted_cell[1:] != sorted_cell[:-1]]]
    representatives = remaining[first]
    diagnostic = {"grouping_voxels": len(representatives), "grouping_budget_exceeded": False,
                  "neighbor_limit": config.object_max_neighbors}
    if len(representatives) > config.max_grouping_voxels:
        diagnostic["grouping_budget_exceeded"] = True
        return [], indices, diagnostic
    tree = cKDTree(representatives)
    k = min(config.object_max_neighbors, len(representatives))
    distances, neighbors = tree.query(representatives, k=k,
                                     distance_upper_bound=config.object_radius_m, workers=1)
    if k == 1:
        distances, neighbors = distances[:, None], neighbors[:, None]
    present = np.isfinite(distances)
    core = present.sum(axis=1) >= config.object_min_neighbors
    core_indices = np.flatnonzero(core)
    if not len(core_indices):
        return [], indices, diagnostic
    core_number = np.full(len(representatives) + 1, -1, dtype=np.int64)
    core_number[core_indices] = np.arange(len(core_indices))
    mapped = core_number[neighbors[core_indices]]
    valid = mapped >= 0
    row = np.broadcast_to(np.arange(len(core_indices))[:, None], mapped.shape)[valid]
    graph = coo_matrix((np.ones(valid.sum(), dtype=np.uint8), (row, mapped[valid])),
                       shape=(len(core_indices), len(core_indices))).tocsr()
    _, component = connected_components(graph, directed=False, return_labels=True)
    voxel_label = np.full(len(representatives), -1, dtype=np.int32)
    voxel_label[core_indices] = component
    border = np.flatnonzero(~core)
    if len(border):
        # A border point belongs to its closest core component, but does not
        # participate in connectivity and therefore cannot bridge two objects.
        distance, nearest = cKDTree(representatives[core_indices]).query(
            representatives[border], k=1, distance_upper_bound=config.object_radius_m)
        valid_border = np.isfinite(distance)
        voxel_label[border[valid_border]] = component[nearest[valid_border]]
    labels = voxel_label[inverse]
    groups, assigned = [], np.zeros(len(indices), dtype=bool)
    for label in np.unique(labels[labels >= 0]):
        mask = labels == label
        if mask.sum() < config.object_min_points or np.count_nonzero(voxel_label == label) < config.object_min_voxels:
            continue
        groups.append(indices[mask])
    groups.sort(key=lambda group: (-len(group), tuple(points[group].min(axis=0))))
    diagnostic["candidate_groups_before_limit"] = len(groups)
    groups = groups[:config.max_object_groups]
    if groups:
        accepted = np.concatenate(groups)
        assigned = np.isin(indices, accepted)
    return groups, indices[~assigned], diagnostic


def build_groups(points, patches, *, gravity_aligned=False, config=None):
    """Suggest room surfaces and partition measured points for human labeling.

    ``config`` accepts SemanticGroupConfig, a dictionary of its fields, or None.
    The returned int32 point_group_ids index the returned groups list, in the
    original input order. No suggestion is a human-reviewed training label.
    """
    if config is None:
        config = SemanticGroupConfig()
    elif isinstance(config, dict):
        config = SemanticGroupConfig(**config)
    if not isinstance(config, SemanticGroupConfig):
        raise ValueError("config must be SemanticGroupConfig or a field dictionary")
    config.validate()
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("points must be a finite Nx3 array in metres")
    if len(points) > config.max_input_points:
        raise ValueError("point count exceeds max_input_points")
    planes = _patches(points, patches)
    suggestions, diagnostic = _structural_suggestions(points, planes, gravity_aligned, config)
    assigned = np.full(len(points), -1, dtype=np.int32)
    records = []
    for patch, label, confidence, reason in suggestions:
        candidates = np.flatnonzero(assigned < 0)
        if not len(candidates):
            break
        # The finite neighborhood of actual support is essential: an infinite
        # wall/floor plane would steal remote or disconnected objects.
        normal_distance = np.abs(np.einsum("ij,j->i", points[candidates] - patch["origin"],
                                            patch["normal"]))
        candidates = candidates[normal_distance <= config.structure_distance_m]
        if not len(candidates):
            continue
        distances, _ = cKDTree(points[patch["indices"]]).query(
            points[candidates], k=1, distance_upper_bound=config.structure_support_radius_m)
        indices = candidates[np.isfinite(distances)]
        if not len(indices):
            continue
        assigned[indices] = len(records)
        records.append((_descriptor(points, indices, "structure", label, confidence, reason,
                                    [patch["id"]]), indices))
    candidates, noise, object_diagnostics = _object_components(points, np.flatnonzero(assigned < 0), config)
    for indices in candidates:
        attached = [patch["id"] for patch in planes
                    if np.intersect1d(indices, patch["indices"], assume_unique=True).size >= 3]
        records.append((_descriptor(points, indices, "object", reason=
            "Grupo por proximidad: puede ser un objeto, parte de uno o varios en contacto. Sin categoría automática.",
            surface_ids=attached), indices))
    if len(noise):
        records.append((_descriptor(points, noise, "unassigned", reason=
            "Puntos dispersos o grupos pequeños sin separación fiable; se conservan para revisión."), noise))
    # Content-derived IDs and deterministic ordering survive shuffled inputs.
    rank = {"structure": 0, "object": 1, "unassigned": 2}
    records.sort(key=lambda item: (rank[item[0]["kind"]], item[0]["suggested_label"] or "",
                                   -item[0]["point_count"], item[0]["id"]))
    assignments = np.full(len(points), -1, dtype=np.int32)
    for index, (_, indices) in enumerate(records):
        if np.any(assignments[indices] >= 0):
            raise RuntimeError("grouping produced overlapping point ownership")
        assignments[indices] = index
    if np.any(assignments < 0):
        raise RuntimeError("grouping lost input points")
    groups = [record for record, _ in records]
    diagnostic.update(object_diagnostics)
    diagnostic.update(config=asdict(config), gravity_aligned=bool(gravity_aligned),
                      input_points=len(points), assigned_points=int(np.count_nonzero(assignments >= 0)),
                      structural_groups=sum(group["kind"] == "structure" for group in groups),
                      object_groups=sum(group["kind"] == "object" for group in groups),
                      unassigned_points=len(noise), confidence_is_calibrated_probability=False,
                      labels_require_human_review=True, learned_model=False,
                      algorithm="bounded_structural_support_and_density_groups_v1")
    return {"groups": groups, "point_group_ids": assignments, "diagnostics": diagnostic}
