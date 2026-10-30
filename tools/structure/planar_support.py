"""Bounded interpolation of measured planar support, retaining open regions."""
import numpy as np
from scipy.spatial import Delaunay, QhullError, cKDTree
import shapely
from shapely.geometry import Polygon


def connected_groups(points, radius):
    """Separate spatially disconnected pieces of a common infinite plane."""
    parent = np.arange(len(points))
    size = np.ones(len(points), dtype=np.int64)

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for first, second in cKDTree(points).query_pairs(radius, output_type="ndarray"):
        first, second = find(first), find(second)
        if first == second:
            continue
        if size[first] < size[second]:
            first, second = second, first
        parent[second] = first
        size[first] += size[second]
    groups = {}
    for index in range(len(points)):
        groups.setdefault(find(index), []).append(index)
    return [np.asarray(group, dtype=np.int64) for group in groups.values()]


def sample_grid(uv, cell_size):
    """One actual measured point per cell; no invented cell-centre vertices."""
    cells = np.floor(uv / cell_size).astype(np.int64)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    center = (cells.astype(float) + 0.5) * cell_size
    distances = np.sum((uv - center) ** 2, axis=1)
    order = np.lexsort((uv[:, 1], uv[:, 0], distances, inverse))
    sorted_cells = inverse[order]
    first = np.r_[True, sorted_cells[1:] != sorted_cells[:-1]]
    return uv[order[first]]


def polygons(geometry):
    if geometry.geom_type == "Polygon":
        yield geometry
    elif hasattr(geometry, "geoms"):
        for child in geometry.geoms:
            yield from polygons(child)


def support_polygons(uv, config):
    """Only connect nearby samples; unlike a convex hull this retains holes."""
    samples = sample_grid(uv, config.mesh_sample_grid_m)
    if len(samples) < 3:
        return []
    try:
        triangles = Delaunay(samples).simplices
    except QhullError:
        return []
    tri = samples[triangles]
    lengths = np.stack([np.linalg.norm(tri[:, (i + 1) % 3] - tri[:, i], axis=1)
                        for i in range(3)], axis=1)
    edge_a, edge_b = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    doubled_area = np.abs(edge_a[:, 0] * edge_b[:, 1] - edge_a[:, 1] * edge_b[:, 0])
    radius = np.prod(lengths, axis=1) / np.maximum(2 * doubled_area, 1e-20)
    keep = ((lengths.max(axis=1) <= config.mesh_max_edge_m)
            & (radius <= config.mesh_max_circumradius_m) & (doubled_area > 1e-10))
    pieces = [Polygon(vertices) for vertices in tri[keep]]
    if not pieces:
        return []
    union = shapely.union_all(pieces)
    result = []
    for original in polygons(union):
        if original.area < config.min_area_m2:
            continue
        # Clipping to the original support is deliberate: simplification alone
        # can extend a face across a concavity or contract an unobserved hole.
        reduced = original.simplify(config.contour_tolerance_m, preserve_topology=True)
        clipped = reduced.intersection(original)
        for part in polygons(clipped):
            if part.area >= config.min_area_m2 and part.is_valid:
                result.append(part)
    return sorted(result, key=lambda item: (-item.area, item.bounds))


def triangulate_polygon(polygon):
    """GEOS constrained triangulation respects the exterior and every hole."""
    geometry = shapely.constrained_delaunay_triangles(polygon)
    coords, faces, lookup = [], [], {}
    for tri in polygons(geometry):
        xy = np.asarray(tri.exterior.coords[:-1], dtype=float)
        if len(xy) != 3 or tri.area <= 1e-12:
            continue
        # The polygon is a subset of the observed-support alpha complex.
        if not polygon.covers(tri):
            continue
        cross = np.cross(np.r_[xy[1] - xy[0], 0], np.r_[xy[2] - xy[0], 0])[2]
        if cross < 0:
            xy = xy[[0, 2, 1]]
        indices = []
        for point in xy:
            key = tuple(point)
            if key not in lookup:
                lookup[key] = len(coords)
                coords.append([point[0], point[1], 0.0])
            indices.append(lookup[key])
        faces.append(indices)
    if not faces:
        return np.empty((0, 3)), np.empty((0, 3), dtype=np.int32), np.empty((0, 2), dtype=np.int32)
    triangles = np.asarray(faces, dtype=np.int32)
    edges = np.sort(np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]],
                                    triangles[:, [2, 0]]]), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    return np.asarray(coords), triangles, unique[counts == 1]


def covered_indices(polygon, uv):
    # Tiny numerical padding includes samples that coincide with the boundary.
    return np.flatnonzero(shapely.intersects_xy(polygon.buffer(1e-9), uv[:, 0], uv[:, 1]))
