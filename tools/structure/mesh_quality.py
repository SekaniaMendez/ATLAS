"""Distance checks against measured points, including unobserved mesh area."""
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def world_mesh(patches):
    vertices, faces, offset = [], [], 0
    for patch in patches:
        local = np.asarray(patch["vertices"], dtype=float)
        triangles = np.asarray(patch["triangles"], dtype=np.int64)
        if not len(local) or not len(triangles):
            continue
        world = np.einsum("nj,ij->ni", local, np.asarray(patch["axes"])) + np.asarray(patch["origin"])
        if not np.isfinite(world).all() or triangles.min() < 0 or triangles.max() >= len(world):
            raise ValueError("Geometría inválida: coordenadas o índices fuera de rango.")
        vertices.append(world)
        faces.append(triangles + offset)
        offset += len(world)
    if not vertices:
        return o3d.geometry.TriangleMesh()
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.concatenate(vertices)),
                                    o3d.utility.Vector3iVector(np.concatenate(faces)))


def distance_to_mesh(points, patches):
    mesh = world_mesh(patches)
    if mesh.is_empty():
        return np.full(len(points), np.inf)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    # CPU raycasting computes point-to-triangle distances, not nearest vertices.
    return scene.compute_distance(o3d.core.Tensor(np.asarray(points, np.float32))).numpy()


def measure_quality(points, patches, *, tolerance_m=0.04, support_radius_m=0.10):
    mesh = world_mesh(patches)
    if mesh.is_empty():
        raise ValueError("No se pudo reconstruir ninguna superficie con soporte suficiente.")
    distances = distance_to_mesh(points, patches)
    vertices, faces = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    triangle_points = vertices[faces]
    area = np.linalg.norm(np.cross(triangle_points[:, 1] - triangle_points[:, 0],
                                   triangle_points[:, 2] - triangle_points[:, 0]), axis=1) * 0.5
    if not np.isfinite(area).all() or np.any(area <= 1e-10):
        raise ValueError("La reconstrucción contiene triángulos degenerados.")
    rng = np.random.default_rng(0)
    selected = rng.choice(len(faces), min(150_000, max(15_000, len(points))),
                          p=area / area.sum())
    triangles = triangle_points[selected]
    a, b = np.sqrt(rng.random(len(selected))), rng.random(len(selected))
    samples = ((1 - a[:, None]) * triangles[:, 0] +
               (a * (1 - b))[:, None] * triangles[:, 1] + (a * b)[:, None] * triangles[:, 2])
    support_distances = cKDTree(points).query(samples, workers=1)[0]
    covered = distances <= tolerance_m
    return {
        "tolerance_m": tolerance_m, "support_radius_m": support_radius_m,
        "point_coverage": float(covered.mean()), "covered_points": int(covered.sum()),
        "unmodeled_points": int((~covered).sum()),
        "point_to_mesh_rmse_m": float(np.sqrt(np.mean(distances ** 2))),
        "point_to_mesh_p95_m": float(np.quantile(distances, .95)),
        "point_to_mesh_max_m": float(distances.max()),
        "mesh_to_samples_p95_m": float(np.quantile(support_distances, .95)),
        "unsupported_area_fraction": float((support_distances > support_radius_m).mean()),
        "area_sample_count": len(samples), "vertices": len(vertices), "triangles": len(faces),
        "surface_area_m2": float(area.sum()),
        "measurement_accuracy_claim": False,
        "limitations": ["Distances compare the derived model to the SLAM measurements, not external ground truth.",
                        "Unobserved space remains unknown; the model is not required to be watertight.",
                        "Area support is sampled and does not prove every point on the mesh was observed."],
    }
