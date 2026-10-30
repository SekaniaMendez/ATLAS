"""Offline measured plane patches and reference frames, without scene semantics.

All transforms map local patch coordinates into the unchanged SLAM map. The
returned objects are hypotheses for this snapshot, not persistent tracked IDs.
"""
import math
import numpy as np
import open3d as o3d

if __package__:
    from .structure_config import StructureConfig
    from .planar_support import connected_groups, support_polygons, triangulate_polygon, covered_indices
else:
    from structure_config import StructureConfig
    from planar_support import connected_groups, support_polygons, triangulate_polygon, covered_indices


def canonical_normal(normal):
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    # Fix sign without assuming the sensor observed the inside or outside.
    axis = int(np.argmax(np.abs(normal)))
    return normal if normal[axis] >= 0 else -normal


def fit_plane(points):
    origin = points.mean(axis=0)
    centered = points - origin
    covariance = np.einsum("ni,nj->ij", centered, centered) / len(points)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    return origin, canonical_normal(eigenvectors[:, 0]), eigenvalues


def plane_axes(normal):
    normal = canonical_normal(normal)
    up = np.array([0.0, 0.0, 1.0])
    if abs(normal[2]) < 0.9:
        x_axis = np.cross(up, normal)
    else:
        x_axis = np.array([1.0, 0.0, 0.0]) - normal * normal[0]
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    return np.column_stack([x_axis, y_axis, normal])


def orientation_kind(normal, tolerance_degrees):
    alignment = abs(float(normal[2]))
    if alignment >= math.cos(math.radians(tolerance_degrees)):
        return "horizontal"
    if alignment <= math.sin(math.radians(tolerance_degrees)):
        return "vertical"
    return "inclined"


def _component_patches(points, indices, normals, config):
    # Refit each disconnected component separately and recheck its support.
    # A plane fitted to the entire RANSAC candidate may conceal a small tilted
    # component, so global residual acceptance is not sufficient here.
    angle_threshold = math.cos(math.radians(config.normal_angle_degrees))
    for _ in range(8):
        measured = points[indices]
        origin, normal, spread = fit_plane(measured)
        distances = np.abs(np.einsum("ij,j->i", measured - origin, normal))
        alignments = np.abs(np.einsum("ij,j->i", normals[indices], normal))
        keep = (distances <= config.distance_threshold_m) & (alignments >= angle_threshold)
        if keep.all():
            break
        indices = indices[keep]
        if len(indices) < config.min_support_points:
            return []
    else:
        return []
    if spread[1] < config.min_extent_m ** 2 / 12:
        return []
    axes = plane_axes(normal)
    uv = np.einsum("ni,ij->nj", measured - origin, axes[:, :2])
    if min(np.ptp(uv, axis=0)) < config.min_extent_m:
        return []
    result = []
    for polygon in support_polygons(uv, config):
        covered = covered_indices(polygon, uv)
        if len(covered) < config.min_support_points:
            continue
        vertices, triangles, boundary = triangulate_polygon(polygon)
        if len(triangles) == 0:
            continue
        center_uv = uv[covered].mean(axis=0)
        local_origin = origin + axes[:, :2] @ center_uv
        vertices[:, :2] -= center_uv
        support = indices[covered]
        residuals = np.einsum("ij,j->i", points[support] - local_origin, normal)
        rmse = float(np.sqrt(np.mean(residuals ** 2)))
        if rmse > config.distance_threshold_m:
            continue
        result.append({
            "id": "", "kind": orientation_kind(normal, config.orientation_angle_degrees),
            "origin": local_origin, "axes": axes.copy(), "normal": normal.copy(),
            "offset": -float(normal @ local_origin), "support_indices": np.sort(support),
            "rmse_m": rmse, "max_residual_m": float(np.max(np.abs(residuals))),
            "area_m2": float(polygon.area), "vertices": vertices, "triangles": triangles,
            "boundary_segments": boundary, "hole_count": len(polygon.interiors),
            "orientation_reference": "map_Z", "semantic_label": None,
            "origin_definition": "projected_support_centroid",
            "support_max_connection_m": config.mesh_max_edge_m,
            "support_max_circumradius_m": config.mesh_max_circumradius_m,
            "surface_model": "bounded_planar_interpolation",
        })
    return result


def extract(points, config=None):
    """Return bounded, disconnected plane patches from an Nx3 metric snapshot.

    The input is never modified. support_indices always index that original
    array. Connectivity and interpolation bounds preserve larger unobserved
    gaps; this is not a watertight or complete reconstruction of the scene.
    """
    config = (config or StructureConfig()).validate()
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("points must be a finite Nx3 array in metres")
    if len(points) < config.min_support_points:
        return []
    order = np.lexsort((points[:, 2], points[:, 1], points[:, 0]))
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[order]))
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=config.normal_radius_m, max_nn=config.normal_max_neighbors))
    normals = np.empty_like(points)
    normals[order] = np.asarray(cloud.normals)
    available = order.copy()
    o3d.utility.random.seed(config.random_seed)
    threshold = math.cos(math.radians(config.normal_angle_degrees))
    patches = []
    for _ in range(config.max_plane_hypotheses):
        if len(available) < config.min_support_points:
            break
        candidate = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[available]))
        coefficients, selected = candidate.segment_plane(
            distance_threshold=config.distance_threshold_m, ransac_n=3,
            num_iterations=config.ransac_iterations, probability=0.999)
        if len(selected) < config.min_support_points:
            break
        selected = np.asarray(selected, dtype=np.int64)
        proposed = available[selected]
        # Remove the full candidate from further RANSAC hypotheses, but only
        # accepted support is labelled. All other points remain for residual
        # reconstruction by the caller, including noisy edges or small patches.
        remaining = np.ones(len(available), dtype=bool)
        remaining[selected] = False
        available = available[remaining]
        normal = canonical_normal(np.asarray(coefficients[:3]))
        proposed = proposed[np.abs(np.einsum("ij,j->i", normals[proposed], normal)) >= threshold]
        if len(proposed) < config.min_support_points:
            continue
        for _ in range(2):
            origin, normal, _ = fit_plane(points[proposed])
            keep = (np.abs(np.einsum("ij,j->i", points[proposed] - origin, normal))
                    <= config.distance_threshold_m)
            proposed = proposed[keep]
            if len(proposed) < config.min_support_points:
                break
        if len(proposed) < config.min_support_points:
            continue
        for group in connected_groups(points[proposed], config.connectivity_radius_m):
            if len(group) >= config.min_support_points:
                patches.extend(_component_patches(points, proposed[group], normals, config))
    patches.sort(key=lambda patch: (-round(patch["area_m2"], 6),
        tuple(np.round(patch["normal"], 6)), tuple(np.round(patch["origin"], 6))))
    patches = patches[:config.max_patches]
    for index, patch in enumerate(patches, 1):
        patch["id"] = f"plane_{index:03d}"
    return patches


def derive_structural_frame(patches, *, gravity_aligned=False):
    """A separate local reference; no geographical heading or map rotation.

    A horizontal plane alone cannot observe yaw. In that case retain the map
    axes and explicitly mark yaw as undefined by structure.
    """
    if not gravity_aligned or not patches:
        return None
    horizontal = [patch for patch in patches if patch["kind"] == "horizontal"]
    vertical = [patch for patch in patches if patch["kind"] == "vertical"]
    origin_patch = max(horizontal or patches, key=lambda patch: patch["area_m2"])
    axes = np.eye(3)
    source_ids = [origin_patch["id"]]
    yaw_observed = False
    if vertical:
        wall = max(vertical, key=lambda patch: patch["area_m2"])
        x_axis = np.array(wall["normal"], dtype=float)
        x_axis[2] = 0
        x_axis /= np.linalg.norm(x_axis)
        x_axis = canonical_normal(x_axis)
        z_axis = np.array([0.0, 0.0, 1.0])
        axes = np.column_stack([x_axis, np.cross(z_axis, x_axis), z_axis])
        if wall["id"] not in source_ids:
            source_ids.append(wall["id"])
        yaw_observed = True
    origin = np.array(origin_patch["origin"], dtype=float)
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = axes, origin
    return {"origin_m": origin, "axes": axes, "T_map_structural": transform,
            "source_patch_ids": source_ids, "gravity_aligned": True,
            "yaw_defined_by_plane": yaw_observed, "heading_is_geographic": False,
            "yaw_status": "plane_normal_sign_convention" if yaw_observed else "map_yaw_retained_unobserved",
            "origin_definition": "projected_support_centroid"}
