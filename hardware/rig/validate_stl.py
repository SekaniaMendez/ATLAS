#!/usr/bin/env python3
"""Validate binary STL topology with Python's standard library.

STL has no unit metadata: dimensions/volumes are interpreted as mm/mm³.
This checks topology, winding and volume, not self-intersections, fit, print
orientation, material properties or mechanical strength.
"""
import argparse
from collections import defaultdict
from io import BytesIO
import json
import math
from pathlib import Path
import struct
import sys


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


class Weld:
    """Deterministic spatial hash; neighboring bins avoid rounding boundaries."""
    def __init__(self, tolerance):
        self.tolerance = tolerance
        self.vertices = []
        self.bins = defaultdict(list)

    def add(self, point):
        key = tuple(math.floor(v/self.tolerance) for v in point)
        candidates = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for index in self.bins.get((key[0]+dx, key[1]+dy, key[2]+dz), ()):
                        delta = sub(point, self.vertices[index])
                        if dot(delta, delta) <= self.tolerance**2:
                            candidates.append(index)
        if candidates:
            return min(candidates)
        index = len(self.vertices)
        self.vertices.append(point)
        self.bins[key].append(index)
        return index


class Components:
    def __init__(self, size):
        self.parents = list(range(size))

    def find(self, index):
        while index != self.parents[index]:
            self.parents[index] = self.parents[self.parents[index]]
            index = self.parents[index]
        return index

    def join(self, a, b):
        a, b = self.find(a), self.find(b)
        if a != b:
            self.parents[max(a, b)] = min(a, b)


def inspect_stream(stream, byte_size, name, tolerance=1e-5, allow_multiple=False):
    report = {"file": name, "valid": False, "units_assumed": "mm",
              "weld_tolerance_mm": tolerance, "errors": []}
    errors = report["errors"]
    if byte_size < 84:
        errors.append("Binary STL shorter than the 84-byte header")
        return report
    header = stream.read(84)
    count = struct.unpack_from("<I", header, 80)[0]
    report["triangles"] = count
    if byte_size != 84 + count*50:
        errors.append("File size does not match binary STL triangle count (ASCII is unsupported)")
        return report
    if not count:
        errors.append("Empty mesh")
        return report
    weld = Weld(tolerance)
    faces = []
    nonfinite = 0
    degenerate = 0
    bounds_min = [math.inf]*3
    bounds_max = [-math.inf]*3
    for _ in range(count):
        record = stream.read(50)
        if len(record) != 50:
            errors.append("File truncated while reading triangles")
            return report
        values = struct.unpack("<12fH", record)
        if not all(math.isfinite(value) for value in values[:12]):
            nonfinite += 1
            continue
        a, b, c = (values[3:6], values[6:9], values[9:12])
        for point in (a, b, c):
            for axis in range(3):
                bounds_min[axis] = min(bounds_min[axis], point[axis])
                bounds_max[axis] = max(bounds_max[axis], point[axis])
        face = tuple(weld.add(point) for point in (a, b, c))
        wa, wb, wc = (weld.vertices[i] for i in face)
        normal = cross(sub(wb, wa), sub(wc, wa))
        if len(set(face)) < 3 or dot(normal, normal) <= tolerance**4:
            degenerate += 1
        faces.append(face)
    report.update(nonfinite_triangles=nonfinite, degenerate_triangles=degenerate,
                  welded_vertices=len(weld.vertices))
    if nonfinite:
        errors.append(f"{nonfinite} triangles contain nonfinite normals or coordinates")
    if degenerate:
        errors.append(f"{degenerate} degenerate triangles after welding")
    if not weld.vertices:
        errors.append("No finite vertices")
        return report
    report.update(bounds_min_mm=bounds_min, bounds_max_mm=bounds_max,
                  dimensions_mm=sub(bounds_max, bounds_min))
    edges = defaultdict(list)
    for index, face in enumerate(faces):
        for a, b in zip(face, face[1:] + face[:1]):
            edges[min(a, b), max(a, b)].append((index, a < b))
    boundary = sum(len(users) == 1 for users in edges.values())
    nonmanifold = sum(len(users) > 2 for users in edges.values())
    winding = sum(len(users) == 2 and users[0][1] == users[1][1]
                  for users in edges.values())
    components = Components(len(faces))
    for users in edges.values():
        for face, _ in users[1:]:
            components.join(users[0][0], face)
    groups = defaultdict(list)
    origin = tuple((lo+hi)/2 for lo, hi in zip(bounds_min, bounds_max))
    for index, face in enumerate(faces):
        a, b, c = (sub(weld.vertices[i], origin) for i in face)
        groups[components.find(index)].append(dot(a, cross(b, c))/6)
    volumes = [math.fsum(parts) for _, parts in sorted(groups.items())]
    report.update(boundary_edges=boundary, nonmanifold_edges=nonmanifold,
                  inconsistent_winding_edges=winding, connected_components=len(groups),
                  component_signed_volumes_mm3=volumes, signed_volume_mm3=math.fsum(volumes),
                  watertight=not (boundary or nonmanifold or degenerate or nonfinite),
                  consistent_winding=not winding)
    if boundary or nonmanifold:
        errors.append(f"Mesh is not closed manifold: {boundary} boundary, {nonmanifold} nonmanifold edges")
    if winding:
        errors.append(f"{winding} shared edges have inconsistent face winding")
    if len(groups) != 1 and not allow_multiple:
        errors.append(f"Expected one connected component, found {len(groups)}")
    if any(volume <= tolerance**3 for volume in volumes):
        errors.append("A component has zero or negative signed volume (inward winding)")
    report["valid"] = not errors
    return report


def inspect_file(path, tolerance=1e-5, allow_multiple=False):
    try:
        with path.open("rb") as stream:
            return inspect_stream(stream, path.stat().st_size, str(path), tolerance, allow_multiple)
    except (OSError, ValueError, OverflowError) as error:
        return {"file": str(path), "valid": False, "errors": [str(error)]}


def self_test():
    vertices = [(0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.)]
    tetra = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]

    def binary(points, faces):
        data = bytearray(80) + struct.pack("<I", len(faces))
        for face in faces:
            data.extend(struct.pack("<12fH", 0, 0, 0,
                                    *(v for i in face for v in points[i]), 0))
        return bytes(data)

    def check(data, multiple=False):
        return inspect_stream(BytesIO(data), len(data), "self-test", allow_multiple=multiple)

    valid = check(binary(vertices, tetra))
    assert valid["valid"] and abs(valid["signed_volume_mm3"] - 1/6) < 1e-12
    assert valid["dimensions_mm"] == (1., 1., 1.)
    assert not check(binary(vertices, tetra[:-1]))["valid"]
    assert check(binary(vertices, [(0, 0, 1)] + tetra[1:]))["degenerate_triangles"] == 1
    flipped = [tuple(reversed(tetra[0]))] + tetra[1:]
    assert check(binary(vertices, flipped))["inconsistent_winding_edges"] == 3
    assert not check(binary(vertices, [tuple(reversed(face)) for face in tetra]))["valid"]
    duplicate = tetra + [tetra[0]]
    assert check(binary(vertices, duplicate))["nonmanifold_edges"] == 3
    assert not check(binary(vertices, tetra) + b"x")["valid"]
    bad = list(vertices); bad[0] = (float("nan"), 0., 0.)
    assert check(binary(bad, tetra))["nonfinite_triangles"] == 3
    two_points = vertices + [(x+3, y, z) for x, y, z in vertices]
    two_faces = tetra + [tuple(i+4 for i in face) for face in tetra]
    two = binary(two_points, two_faces)
    assert not check(two)["valid"] and check(two, True)["valid"]
    weld = Weld(1e-5)
    assert weld.add((.99999e-5, 0, 0)) == weld.add((1.00001e-5, 0, 0))
    print(json.dumps({"self_test": "passed"}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path, help="Binary STL files or folders (recursive)")
    parser.add_argument("--tolerance-mm", type=float, default=1e-5)
    parser.add_argument("--allow-multiple", action="store_true", help="Allow multiple closed components")
    parser.add_argument("--json-output", type=Path, help="Also save the JSON report to a file")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.tolerance_mm) or args.tolerance_mm <= 0:
        parser.error("--tolerance-mm must be positive and finite")
    if args.self_test:
        self_test()
        if not args.paths:
            return 0
    if not args.paths:
        parser.error("Supply at least one STL file or folder")
    files = set()
    for path in args.paths:
        if path.is_dir():
            files.update(p for p in path.rglob("*") if p.is_file() and p.suffix.lower() == ".stl")
        else:
            files.add(path)
    reports = [inspect_file(path, args.tolerance_mm, args.allow_multiple) for path in sorted(files)]
    report = {"valid": bool(reports) and all(r["valid"] for r in reports), "files": reports,
              "limitations": "STL units assumed mm. Does not detect self-intersections, fit, or mechanical strength."}
    if not files:
        report["error"] = "No STL files found"
    output = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    print(output, end="")
    if args.json_output:
        args.json_output.write_text(output)
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())
