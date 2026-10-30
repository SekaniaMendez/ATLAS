#!/usr/bin/env python3
"""Exercise the real C++ live geometry TCP stream with saved or synthetic data.

No Livox SDK is started. Requires the atlas_geometry_preview executable and local
socket permission. All generated fixtures, logs and reports stay below build/.
Run: python3 tests/ios/test_live_geometry_stream.py [--record-ply output/SESSION/mapa.ply]
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
HEADER = struct.Struct("<IHHIIQII")
PREFIX = struct.Struct("<HHIIIQIf")
SURFACE = struct.Struct("<II3f9fffIIIII")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fixture(path):
    points = [(x * .05, y * .05, .5) for x in range(-20, 21) for y in range(-20, 21)
              if abs(x) > 4 or abs(y) > 4]
    path.write_text("ply\nformat ascii 1.0\nelement vertex " + str(len(points)) +
                    "\nproperty float x\nproperty float y\nproperty float z\nend_header\n" +
                    "".join(f"{x} {y} {z}\n" for x, y, z in points))


def validate_invalid_inputs(binary, work):
    header = ("ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\n"
              "property float y\nproperty float z\nend_header\n")
    cases = (
        header + "0 0 0\n",  # Truncated source.
        header + "0 0 0\n1 0 0\nnan 1 0\n",
        header + "0 0 0\n1 0 0\n0 1 0\n2 2 2\n",  # Extra source rows.
        header.replace("vertex 3", "vertex 2000001"),
        header.replace("property float z", "property float other") + "0 0 0\n1 0 0\n0 1 0\n",
        header.replace("ascii", "binary_little_endian"),
    )
    for index, contents in enumerate(cases):
        path = work / f"invalid-{index}.ply"
        path.write_text(contents)
        original = sha(path)
        result = subprocess.run([str(binary), "--input", str(path), "--seconds", ".05"],
                                capture_output=True, text=True, timeout=5)
        assert result.returncode == 1 and "saved_points=" not in result.stdout, result.stdout + result.stderr
        assert sha(path) == original
    return len(cases)


def receive(sock, count):
    data = bytearray()
    while len(data) < count:
        block = sock.recv(count - len(data))
        if not block:
            raise RuntimeError("Preview stream ended before the test completed")
        data.extend(block)
    return bytes(data)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def geometry(payload, header_count):
    assert PREFIX.size == 32 and SURFACE.size == 84
    assert len(payload) >= PREFIX.size
    version, flags, count, vertices, indices, revision, input_count, ms = PREFIX.unpack_from(payload)
    assert version == 1 and flags & 1 and flags & ~3 == 0
    assert count == header_count and count <= 32 and vertices <= 30000 and indices <= 90000
    assert len(payload) == 32 + count * 84 + vertices * 12 + indices * 4
    assert math.isfinite(ms) and ms >= 0
    vertex_start = 32 + count * 84
    index_start = vertex_start + vertices * 12
    next_vertex = next_index = 0
    ids = set()
    for item in range(count):
        values = SURFACE.unpack_from(payload, 32 + item * 84)
        identity, kind = values[:2]
        origin, axes = values[2:5], values[5:14]
        area, rmse, support, first_vertex, vertex_count, first_index, index_count = values[14:]
        assert identity > 0 and identity not in ids and kind in (1, 2, 3, 4)
        ids.add(identity)
        assert all(math.isfinite(x) for x in (*origin, *axes, area, rmse))
        assert area > 0 and rmse >= 0 and support > 0
        basis = [axes[i:i + 3] for i in (0, 3, 6)]
        for i in range(3):
            for j in range(3):
                assert abs(dot(basis[i], basis[j]) - (1 if i == j else 0)) < .002
        a, b, c = basis
        determinant = dot(a, (b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2],
                              b[0] * c[1] - b[1] * c[0]))
        assert determinant > .998
        assert first_vertex == next_vertex and first_index == next_index
        assert vertex_count >= 3 and index_count >= 3 and index_count % 3 == 0
        assert first_vertex + vertex_count <= vertices and first_index + index_count <= indices
        coords = struct.unpack_from("<" + "f" * (vertex_count * 3), payload, vertex_start + first_vertex * 12)
        faces = struct.unpack_from("<" + "I" * index_count, payload, index_start + first_index * 4)
        assert all(math.isfinite(x) for x in coords) and all(i < vertex_count for i in faces)
        for f in range(0, len(faces), 3):
            assert len(set(faces[f:f + 3])) == 3
        next_vertex += vertex_count
        next_index += index_count
    assert next_vertex == vertices and next_index == indices
    return {"surfaces": count, "vertices": vertices, "triangles": indices // 3,
            "revision": revision, "input_points": input_count, "processing_ms": ms,
            "truncated": bool(flags & 2)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record-ply", type=Path)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/mac-release/apps/atlas_geometry_preview")
    args = parser.parse_args()
    base = ROOT / "build/live-geometry-stream-tests"
    base.mkdir(exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix="run-", dir=base))
    source = args.record_ply.resolve() if args.record_ply else work / "measured-plane.ply"
    if not args.record_ply:
        fixture(source)
    original_sha = sha(source)
    rejected_inputs = validate_invalid_inputs(args.binary.resolve(), work)
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    command = [str(args.binary.resolve()), "--input", str(source), "--port", str(port),
               "--seconds", "24", "--exercise-clears"]
    phase = "unknown"
    seen = {name: False for name in ("full_geometry", "empty_clear", "reappearance_geometry",
                                    "lost_clear", "recovered_geometry", "world_points")}
    geometry_events = []
    with (work / "server.log").open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while True:
                try:
                    sock = socket.create_connection(("127.0.0.1", port), timeout=.5)
                    break
                except OSError:
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError("Preview did not open its test port; inspect " + str(work / "server.log"))
                    time.sleep(.05)
            with sock:
                sock.settimeout(4)
                deadline = time.monotonic() + 21
                last_geometry_revision = 0
                last_surface_count = None
                while not all(seen.values()) and time.monotonic() < deadline:
                    magic, version, kind, size, _, timestamp, count, flags = HEADER.unpack(receive(sock, 32))
                    assert magic == 0x534C5441 and version == 1 and size <= 1024 * 1024
                    payload = receive(sock, size)
                    if kind == 2 and flags == 1:
                        status_version, tracking, fatal, _, _, text_length = struct.unpack_from("<HBBQQI", payload)
                        assert status_version == 1 and len(payload) == 24 + text_length and not fatal
                        message = payload[24:].decode()
                        phase = message.split(": ", 1)[1].split(";", 1)[0]
                        assert tracking == (3 if phase == "tracking_lost" else 2)
                        # An empty geometry replacement may precede its lost
                        # status by one sender wakeup; either order is valid.
                        if phase == "tracking_lost" and last_surface_count == 0:
                            seen["lost_clear"] = True
                    elif kind == 2 and flags == 2:
                        result = geometry(payload, count)
                        assert result["revision"] >= last_geometry_revision
                        last_geometry_revision = result["revision"]
                        last_surface_count = result["surfaces"]
                        geometry_events.append({"phase": phase, "timestamp_ns": timestamp, **result})
                        if phase == "full" and result["surfaces"]:
                            seen["full_geometry"] = True
                        if phase == "empty" and not result["surfaces"]:
                            seen["empty_clear"] = True
                        if phase == "reappearance" and result["surfaces"]:
                            seen["reappearance_geometry"] = True
                        if phase == "tracking_lost":
                            assert not result["surfaces"], "Geometry remained after lost tracking"
                            seen["lost_clear"] = True
                        if phase == "recovered" and result["surfaces"]:
                            seen["recovered_geometry"] = True
                    elif kind == 10:
                        assert flags & 1 and size == count * 16 and count <= 50000
                        seen["world_points"] |= count > 0
                assert all(seen.values()), "Missing lifecycle events: " + str(seen)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                code = process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
                raise
    assert code == 0, f"Preview exited with {code}"
    assert sha(source) == original_sha, "Recorded source changed"
    report = {"passed": True, "mode": "saved measurements, C++ worker, real local TCP",
              "live_lidar_started": False, "source": str(source), "source_sha256": original_sha,
              "source_unchanged": True, "lifecycle": seen, "geometry_events": geometry_events,
              "invalid_inputs_rejected_before_listening": rejected_inputs,
              "command": command, "server_exit": code}
    (work / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"passed": True, "report": str(work / "validation.json"), "events": len(geometry_events),
                      "lifecycle": seen}, indent=2))


if __name__ == "__main__":
    main()
