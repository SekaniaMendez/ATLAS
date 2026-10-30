#!/usr/bin/env python3
"""Capture a new SLAM session and export compact geometry after graceful stop.

This module uses only the Python standard library. The geometry analyzer runs in
its separate environment. It never accepts an existing PLY as a deletion target.
"""

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile


def announce(message):
    print(message, flush=True)


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def write_status(path, status):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(status, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def exit_status(code):
    return code if code >= 0 else 128 - code


class InterruptRelay:
    """The terminal signals this runner; each child owns a separate process group."""

    def __init__(self):
        self.child = None
        self.forwarded = False
        self.cancel_before_start = False
        self.previous = {}

    def __enter__(self):
        signals = [signal.SIGINT, signal.SIGTERM]
        if hasattr(signal, "SIGHUP"):
            signals.append(signal.SIGHUP)
        for sig in signals:
            self.previous[sig] = signal.signal(sig, self.interrupt)
        return self

    def __exit__(self, *_):
        for sig, handler in self.previous.items():
            signal.signal(sig, handler)

    def interrupt(self, _signal, _frame):
        if self.child is None:
            self.cancel_before_start = True
            return
        if self.forwarded:
            os.write(2, b"ATLAS: esperando el cierre del proceso; se conserva la captura.\n")
            return
        self.forwarded = True
        # The probe installs SIGINT, but not SIGTERM. Both request its clean stop.
        if self.child.poll() is None:
            try:
                os.killpg(self.child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def run(self, command, cwd, env=None):
        if self.cancel_before_start:
            raise InterruptedError("Sesión interrumpida antes de iniciar el siguiente proceso")
        self.forwarded = False
        self.child = subprocess.Popen(command, cwd=cwd, env=env, start_new_session=True)
        # A signal can arrive between the preflight check and Popen returning.
        if self.cancel_before_start:
            self.cancel_before_start = False
            self.interrupt(signal.SIGINT, None)
        try:
            return exit_status(self.child.wait())
        finally:
            self.child = None


def checked_paths(root):
    paths = {
        "probe": root / "build/mac-release/apps/atlas_livox_probe",
        "config": root / "config/livox/mid360.json",
        "geometry_python": root / "build/structure-venv/bin/python",
        "analyzer": root / "tools/structure/analyze.py",
    }
    for name, path in paths.items():
        if not path.is_file() or (name in ("probe", "geometry_python") and not os.access(path, os.X_OK)):
            raise RuntimeError(f"Falta {name}: {path}")
    return paths


def geometry_ready(directory, source_sha256):
    for name in ("scene.glb", "scene.json", "quality.json", "viewer.html"):
        path = directory / name
        if not path.is_file() or path.is_symlink() or path.stat().st_size == 0:
            return False
    with (directory / "quality.json").open(encoding="utf-8") as stream:
        quality = json.load(stream)
    return (isinstance(quality, dict) and quality.get("safe_to_discard_source") is True and
            quality.get("source_sha256") == source_sha256)


def run_session(root, keep_source=False):
    root = Path(root).resolve()
    paths = checked_paths(root)
    output_root = root / "output"
    output_root.mkdir(exist_ok=True)
    session = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("%Y%m%d-%H%M%S-"), dir=output_root))
    source = session / "mediciones.ply"
    trajectory = session / "trayectoria.csv"
    metadata = session / "slam_session.txt"
    geometry = session / "geometria"
    status_path = session / "session.json"
    command = [str(paths["probe"]), str(paths["config"]), "--slam",
               "--ply", str(source), "--trajectory", str(trajectory),
               "--session-report", str(metadata)]
    if keep_source:
        command += ["--map", str(session / "mapa")]
    status = {"schema_version": 1, "state": "capturing", "keep_source": keep_source,
              "capture_command": command, "source_ply": str(source),
              "source_deleted": False, "probe_exit": None, "analyzer_exit": None}
    write_status(status_path, status)
    announce(f"\nSesión: {session}")
    announce("Mantén el MID-360 quieto hasta ver state=tracking.")
    announce("Ctrl+C termina la captura y comienza la creación de geometría. Espera a que finalice.")
    with InterruptRelay() as relay:
        try:
            status["probe_exit"] = relay.run(command, root)
            status["state"] = "capture_finished"
            write_status(status_path, status)
            if not source.is_file() or source.is_symlink() or source.stat().st_size == 0:
                status["state"] = "no_map"
                write_status(status_path, status)
                announce("ATLAS no produjo una nube válida; no se inició la reconstrucción.")
                return status["probe_exit"] or 1
            if not metadata.is_file():
                raise RuntimeError("Falta slam_session.txt; se conserva el PLY para recuperar la sesión")
            source_sha = digest(source)
            status["source_sha256"] = source_sha
            status["source_bytes"] = source.stat().st_size
            status["state"] = "reconstructing"
            analysis = [str(paths["geometry_python"]), str(paths["analyzer"]),
                        "--input", str(source), "--output", str(geometry),
                        "--session-metadata", str(metadata)]
            status["analysis_command"] = analysis
            write_status(status_path, status)
            announce("\nCaptura cerrada. Creando superficies, marcos y modelo compacto...")
            analysis_env = os.environ.copy()
            analysis_env.setdefault("OMP_NUM_THREADS", "4")
            analysis_env.setdefault("OPENBLAS_NUM_THREADS", "1")
            status["analyzer_exit"] = relay.run(analysis, root, env=analysis_env)
            ready = status["analyzer_exit"] == 0 and geometry_ready(geometry, source_sha)
            # Only discard the source generated in this brand-new session, after
            # both producers succeeded and the source is still byte-identical.
            safe = (ready and status["probe_exit"] == 0 and
                    source.is_file() and not source.is_symlink() and digest(source) == source_sha)
            status["quality_passed"] = ready
            if safe and not keep_source:
                source.unlink()
                status["source_deleted"] = True
            status["state"] = "complete" if safe else "needs_review"
            write_status(status_path, status)
            if status["analyzer_exit"] == 0:
                announce(f"Modelo: {geometry / 'scene.glb'}")
                announce(f"Marcos y superficies: {geometry / 'scene.json'}")
                announce(f"Visor: {geometry / 'viewer.html'}")
            if status["source_deleted"]:
                announce("Modelo validado; se eliminó únicamente el PLY intermedio de esta sesión.")
            else:
                announce(f"Mediciones conservadas: {source}")
                if not safe:
                    announce("La sesión requiere revisión; consulta session.json y geometria/quality.json.")
            return status["probe_exit"] or status["analyzer_exit"] or (0 if safe else 2)
        except (OSError, ValueError, RuntimeError, InterruptedError) as error:
            status["state"] = "interrupted" if isinstance(error, InterruptedError) else "failed"
            status["error"] = str(error)
            write_status(status_path, status)
            announce(f"ATLAS: {error}")
            if source.exists():
                announce(f"Se conservan las mediciones: {source}")
            return status["probe_exit"] or (130 if isinstance(error, InterruptedError) else 1)


def main():
    parser = argparse.ArgumentParser(description="Captura SLAM y exporta geometría al terminar con Ctrl+C.")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--keep-source", action="store_true", help="Conserva también PLY y archivo de voxels.")
    args = parser.parse_args()
    try:
        return run_session(args.root, args.keep_source)
    except (OSError, RuntimeError) as error:
        print(f"ATLAS: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
