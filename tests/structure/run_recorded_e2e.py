#!/usr/bin/env python3
"""Run an optional offline session integration using a recorded capture.

Usage: python3 tests/structure/run_recorded_e2e.py [--capture-dir PATH]

This is deliberately outside unittest discovery: it uses the installed geometry
environment and real analyzer, with a simulated probe that copies saved inputs
only after SIGINT. It never opens ports or starts the Livox SDK. All generated
files stay under build/geometry-session-e2e/, including validation.json and logs.
The original PLY, trajectory and session metadata must remain byte-identical.
"""

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "build/geometry-session-e2e"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def capture_sources(directory):
    candidates = {
        "ply": ("mapa.ply", "mediciones.ply", "map.ply"),
        "metadata": ("slam_session.txt", "mapa/slam_session.txt", "map/slam_session.txt"),
        "trajectory": ("trayectoria.csv", "trajectory.csv"),
    }
    sources = {}
    for name, relatives in candidates.items():
        paths = [directory / relative for relative in relatives]
        sources[name] = next((path for path in paths if path.is_file()), None)
        if sources[name] is None:
            raise FileNotFoundError(f"Missing {name} in {directory}; tried: {', '.join(relatives)}")
    return sources


STUB = r'''
import json, os, signal, shutil, sys, time
from pathlib import Path
root = Path(__file__).resolve().parents[3]
inputs = json.loads((root / 'probe-fixture.json').read_text())
received = []
def stop(sig, frame):
    received.append(sig)
signal.signal(signal.SIGINT, stop)
(root / 'probe-ready.json').write_text(json.dumps({'pid': os.getpid()}))
deadline = time.monotonic() + 20
while not received and time.monotonic() < deadline:
    time.sleep(.01)
if not received:
    raise SystemExit(3)
args = sys.argv[1:]
assert '--slam' in args and '--map' not in args
for option, source_key in [('--ply', 'ply'), ('--trajectory', 'trajectory'), ('--session-report', 'metadata')]:
    target = Path(args[args.index(option) + 1])
    assert not target.exists(), 'Refusing to overwrite an existing session output'
    assert target.resolve().is_relative_to(root / 'output'), 'Output escaped fixture session'
    shutil.copyfile(inputs[source_key], target)
(root / 'probe-stopped.json').write_text(json.dumps({'signals': received, 'copied_only_saved_files': True}))
raise SystemExit(0)
'''


def validate_glb(path):
    data = path.read_bytes()
    magic, version, length = struct.unpack_from("<4sII", data)
    assert magic == b"glTF" and version == 2 and length == len(data)
    json_length, json_type = struct.unpack_from("<I4s", data, 12)
    assert json_type == b"JSON"
    document = json.loads(data[20:20 + json_length].rstrip(b" \0"))
    bin_start = 20 + json_length
    bin_length, bin_type = struct.unpack_from("<I4s", data, bin_start)
    assert bin_type == b"BIN\0" and bin_start + 8 + bin_length == len(data)
    assert document["buffers"][0]["byteLength"] <= bin_length
    for view in document["bufferViews"]:
        assert view.get("byteOffset", 0) + view["byteLength"] <= bin_length
    for mesh in document["meshes"]:
        for primitive in mesh["primitives"]:
            assert primitive.get("mode", 4) == 4
            assert document["accessors"][primitive["attributes"]["POSITION"]]["count"] > 0
            count = document["accessors"][primitive["indices"]]["count"]
            assert count > 0 and count % 3 == 0
    return {"meshes": len(document["meshes"]), "nodes": len(document["nodes"]), "bytes": len(data)}


def run_supervisor(fixture, log_path):
    command = [sys.executable, str(ROOT / "tools/structure/run_session.py"), "--root", str(fixture)]
    environment = os.environ.copy()
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=environment)
        try:
            deadline = time.monotonic() + 10
            ready = fixture / "probe-ready.json"
            while not ready.exists() and time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f"Supervisor stopped before simulated probe became ready; see {log_path}")
                time.sleep(.01)
            if not ready.exists():
                raise RuntimeError(f"Simulated probe did not become ready; see {log_path}")
            process.send_signal(signal.SIGINT)
            return process.wait(timeout=180)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


def run(capture):
    sources = capture_sources(capture)
    originals = {key: sha(path) for key, path in sources.items()}
    BASE.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("run-%Y%m%d-%H%M%S-"), dir=BASE))
    fixture = work / "fixture"
    (fixture / "build/mac-release/apps").mkdir(parents=True)
    (fixture / "build/structure-venv").symlink_to(ROOT / "build/structure-venv", target_is_directory=True)
    (fixture / "tools").mkdir()
    (fixture / "tools/structure").symlink_to(ROOT / "tools/structure", target_is_directory=True)
    (fixture / "config/livox").mkdir(parents=True)
    shutil.copyfile(ROOT / "config/livox/mid360.json", fixture / "config/livox/mid360.json")
    (fixture / "probe-fixture.json").write_text(json.dumps({key: str(path) for key, path in sources.items()}))
    probe = fixture / "build/mac-release/apps/atlas_livox_probe"
    probe.write_text("#!" + sys.executable + "\n" + STUB)
    probe.chmod(0o755)
    started = time.monotonic()
    try:
        code = run_supervisor(fixture, work / "session.log")
    finally:
        assert all(sha(path) == originals[key] for key, path in sources.items()), "Original capture changed"

    statuses = list((fixture / "output").glob("*/session.json"))
    assert len(statuses) == 1
    status = json.loads(statuses[0].read_text())
    session = statuses[0].parent
    geometry = session / "geometria"
    quality = json.loads((geometry / "quality.json").read_text())
    description = json.loads((geometry / "scene.json").read_text())
    assert status["probe_exit"] == 0 and status["analyzer_exit"] == 0
    assert quality["source_sha256"] == status["source_sha256"] == originals["ply"]
    assert type(quality["safe_to_discard_source"]) is bool
    safe = quality["safe_to_discard_source"] is True
    assert code == (0 if safe else 2)
    assert status["state"] == ("complete" if safe else "needs_review")
    assert status["quality_passed"] is safe
    assert status["source_deleted"] is safe
    source = session / "mediciones.ply"
    assert Path(status["source_ply"]) == source
    if safe:
        assert not source.exists() and not source.is_symlink()
    else:
        assert source.is_file() and not source.is_symlink() and sha(source) == originals["ply"]
    assert sha(session / "slam_session.txt") == originals["metadata"]
    assert sha(session / "trayectoria.csv") == originals["trajectory"]
    assert not (session / "mapa").exists(), "Default session must not create a voxel archive"
    for name in ("scene.glb", "scene.json", "quality.json", "viewer.html"):
        artifact = geometry / name
        assert artifact.is_file() and not artifact.is_symlink() and artifact.stat().st_size > 0
    assert description["surfaces"]
    stopped = json.loads((fixture / "probe-stopped.json").read_text())
    assert stopped["signals"] == [signal.SIGINT]
    assert all(sha(path) == originals[key] for key, path in sources.items())

    report = {
        "passed": True,
        "mode": "offline supervisor + simulated probe + real analyzer and venv",
        "capture_dir": str(capture),
        "network_or_lidar_sdk_started": False,
        "supervisor_exit": code,
        "session_state": status["state"],
        "probe_exit": status["probe_exit"],
        "analyzer_exit": status["analyzer_exit"],
        "ctrl_c_received_by_probe": stopped["signals"],
        "original_hashes_unchanged": True,
        "copied_source_retained": source.exists(),
        "copied_source_deleted": status["source_deleted"],
        "source_sha256": originals["ply"],
        "quality_source_hash_matches": True,
        "point_coverage": quality["point_coverage"],
        "safe_to_discard_source": safe,
        "planar_surfaces": quality["planar_surfaces"],
        "freeform_surfaces": quality["freeform_surfaces"],
        "triangles": quality["triangles"],
        "model_bytes": quality["model_bytes"],
        "compression_ratio": quality["compression_ratio"],
        "glb_validation": validate_glb(geometry / "scene.glb"),
        "elapsed_s": time.monotonic() - started,
        "session": str(session),
        "artifacts": {name: str(geometry / name) for name in ("scene.glb", "scene.json", "quality.json", "viewer.html")},
        "limitations": [
            "Network preflight was not run because this test must not touch ports or hardware.",
            "The simulated probe copies existing measurements after SIGINT; it does not validate live acquisition.",
        ],
    }
    (work / "validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    (BASE / "latest.json").write_text(json.dumps({"validation": str(work / "validation.json"),
                                               "log": str(work / "session.log")}, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--capture-dir", type=Path, default=ROOT / "output/20261028-172733",
                        help="Saved session containing a PLY, trajectory CSV and slam_session.txt.")
    args = parser.parse_args()
    run(args.capture_dir.expanduser().resolve())


if __name__ == "__main__":
    main()
