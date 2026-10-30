"""Bounded freeform interpolation of measured details, with checked reduction.

This is an open surface hypothesis, not a watertight reconstruction. Gaps below
the local connection radius can be interpolated; isolated points remain unknown.
"""
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

if __package__:
    from .mesh_quality import distance_to_mesh
else:
    from mesh_quality import distance_to_mesh


def _mesh(vertices, faces):
    """Compact and deduplicate faces, including reversed winding duplicates."""
    faces = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
    if not len(faces):
        return o3d.geometry.TriangleMesh()
    vertices, inverse = np.unique(np.asarray(vertices), axis=0, return_inverse=True)
    faces = inverse[faces]
    _, unique = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    faces = faces[np.sort(unique)]
    tri = vertices[faces]
    area2 = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    result = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),
                                     o3d.utility.Vector3iVector(faces[area2 > 2e-6]))
    result.remove_unreferenced_vertices()
    return result


def _scene(mesh):
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return scene


def _distance(scene, points):
    return scene.compute_distance(o3d.core.Tensor(np.asarray(points, np.float32))).numpy()


def _probes(mesh):
    triangles = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)]
    return np.concatenate((triangles, (triangles + np.roll(triangles, 1, axis=1)) / 2,
                           triangles.mean(axis=1)[:, None]), axis=1)


def _bounded_faces(vertices, faces, max_edge_m, measured_tree=None):
    """Every accepted triangle stays within max_edge/2 of some measurement.

    The covering radius of the triangle's vertices is its circumradius when
    acute, or half its longest edge otherwise. Adding each vertex's distance
    to an actual sample conservatively bounds mesh-to-measurement distance.
    """
    triangles = vertices[faces]
    lengths = np.sort(np.stack([np.linalg.norm(triangles[:, a] - triangles[:, b], axis=1)
                               for a, b in ((0, 1), (1, 2), (2, 0))], axis=1), axis=1)
    area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                   triangles[:, 2] - triangles[:, 0]), axis=1) / 2
    circumradius = np.prod(lengths, axis=1) / (4 * np.maximum(area, 1e-15))
    covering = np.where(lengths[:, 2] ** 2 >= np.sum(lengths[:, :2] ** 2, axis=1),
                        lengths[:, 2] / 2, circumradius)
    if measured_tree is not None:
        covering += measured_tree.query(vertices, workers=1)[0][faces].max(axis=1)
    return ((lengths[:, 2] <= max_edge_m + 1e-10) & (area > 1e-6)
            & (covering <= max_edge_m / 2 + 1e-10))


def _decimate(mesh):
    # Quadric error is not a metric distance bound. All results are checked below.
    return mesh.simplify_quadric_decimation(max(100, len(mesh.triangles) // 4),
                                           maximum_error=1e-6, boundary_weight=100.0)


def _simplify_bounded(reference, points, residual, tolerance_m, max_edge_m):
    """Restore measured triangles wherever reduction loses points or surface.

    Reduction is accepted only after point coverage, bidirectional surface probes,
    local support, maximum edge length and area checks. Otherwise retain reference.
    """
    limit = min(.015, tolerance_m * .375)
    before_scene = _scene(reference)
    before_probes = _probes(reference)
    before_area = reference.get_surface_area()
    info = {"accepted": False, "reference_triangles": len(reference.triangles),
            "reference_area_m2": before_area, "surface_probe_tolerance_m": limit,
            "validation_samples": "triangle_vertices_edge_midpoints_and_centroids",
            "topology_preservation_claim": False}
    if len(reference.triangles) <= 100:
        return reference, info
    candidate = _decimate(reference)
    vertices, faces = np.asarray(candidate.vertices), np.asarray(candidate.triangles)
    if not len(faces) or not np.isfinite(vertices).all():
        return reference, info
    keep = _bounded_faces(vertices, faces, max_edge_m, cKDTree(points))
    candidate_probes = _probes(candidate)
    keep &= _distance(before_scene, candidate_probes.reshape(-1, 3)).reshape(-1, 7).max(axis=1) <= limit
    candidate = _mesh(vertices, faces[keep])
    if not len(candidate.triangles):
        return reference, info
    candidate_scene = _scene(candidate)
    # Preserve the intervening measured surface, not just discrete sample points.
    restore = _distance(candidate_scene, before_probes.reshape(-1, 3)).reshape(-1, 7).max(axis=1) > limit
    closest = before_scene.compute_closest_points(o3d.core.Tensor(np.asarray(points, np.float32)))
    before_distance = np.linalg.norm(points - closest["points"].numpy(), axis=1)
    covered = residual & (before_distance <= tolerance_m)
    lost = covered & (_distance(candidate_scene, points) > tolerance_m)
    restore[closest["primitive_ids"].numpy()[lost]] = True
    ref_vertices, ref_faces = np.asarray(reference.vertices), np.asarray(reference.triangles)
    vertices, faces = np.asarray(candidate.vertices), np.asarray(candidate.triangles)
    repaired = _mesh(np.concatenate((vertices, ref_vertices)),
                     np.concatenate((faces, ref_faces[restore] + len(vertices))))
    after_scene = _scene(repaired)
    forward = float(_distance(after_scene, before_probes.reshape(-1, 3)).max())
    reverse = float(_distance(before_scene, _probes(repaired).reshape(-1, 3)).max())
    area_change = abs(repaired.get_surface_area() / before_area - 1)
    if (len(repaired.triangles) >= len(reference.triangles) or area_change > .01
            or forward > limit + 1e-6 or reverse > limit + 1e-6
            or np.any(_distance(after_scene, points[covered]) > tolerance_m + 1e-6)):
        return reference, info
    info.update(accepted=True, restored_triangles=int(restore.sum()),
                relative_area_change=area_change, reference_to_reduced_max_probe_m=forward,
                reduced_to_reference_max_probe_m=reverse)
    return repaired, info


def reconstruct_residual(points, patches, *, tolerance_m=0.04, max_edge_m=0.20):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Residual reconstruction requires finite N×3 measurements.")
    if not all(np.isfinite(x) and x > 0 for x in (tolerance_m, max_edge_m)):
        raise ValueError("Residual reconstruction distances must be finite and positive.")
    if len(points) < 3:
        return []
    distances = distance_to_mesh(points, patches)
    residual = distances > tolerance_m
    if residual.sum() < 3:
        return []
    tree = cKDTree(points)
    # The halo provides measured vertices at the planar/freeform boundary.
    neighborhoods = tree.query_ball_point(points[residual], max_edge_m * .6, workers=1)
    indices = np.unique(np.concatenate(neighborhoods)).astype(np.int64)
    local = points[indices]
    passes = [(1., [.15, .25, .375, .5]), (1., [.275, .425, .6]),
              (.6, [.15, .25, .375, .5]), (.4, [.15, .25, .375, .5])]
    faces, missing = [], residual.copy()
    cloud, previous_radius = None, None
    for normal_radius, radii in passes:
        if not missing[indices].any():
            break
        if normal_radius != previous_radius:
            cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(local))
            cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
                radius=max_edge_m * normal_radius, max_nn=40))
            try:
                cloud.orient_normals_consistent_tangent_plane(min(20, len(indices) - 1))
            except RuntimeError:
                # This can reduce BPA connectivity; unsupported regions stay absent.
                cloud.orient_normals_towards_camera_location(np.zeros(3))
            previous_radius = normal_radius
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            cloud, o3d.utility.DoubleVector(np.asarray(radii) * max_edge_m))
        proposed = np.asarray(mesh.triangles)
        if not len(proposed):
            continue
        keep = _bounded_faces(local, proposed, max_edge_m) & missing[indices[proposed]].any(axis=1)
        if not keep.any():
            continue
        faces.append(proposed[keep].copy())
        measured_mesh = _mesh(local, np.concatenate(faces))
        missing = residual & (_distance(_scene(measured_mesh), points) > tolerance_m)
    if not faces:
        return []
    faces = np.concatenate(faces)
    # Only vertices actually used in accepted measured triangles are evidence.
    support = indices[np.unique(faces)]
    reference = _mesh(local, faces)
    mesh, simplification = _simplify_bounded(reference, points, residual, tolerance_m, max_edge_m)
    support = support[_distance(_scene(mesh), points[support]) <= tolerance_m + 1e-6]
    vertices = np.asarray(mesh.vertices)
    origin = vertices.mean(axis=0)
    return [{"id": "detail_001", "kind": "freeform", "origin": origin,
             "axes": np.eye(3), "normal": np.zeros(3), "offset": None,
             "support_indices": np.sort(support), "rmse_m": None,
             "area_m2": float(mesh.get_surface_area()), "vertices": vertices - origin,
             "triangles": np.asarray(mesh.triangles).copy(),
             "boundary_segments": np.empty((0, 2), dtype=int),
             "surface_model": "bounded_measured_ball_pivoting",
             "support_max_connection_m": max_edge_m,
             "mesh_to_measurement_bound_m": max_edge_m / 2,
             "simplification": simplification}]
