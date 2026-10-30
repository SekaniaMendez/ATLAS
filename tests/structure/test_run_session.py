"""Exercise capture shutdown and source-retention policy with local stub programs.

No network sockets, LiDAR SDK or live sensor processes are started.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest


RUNNER = Path(__file__).resolve().parents[2] / "tools/structure/run_session.py"

PROBE = r'''
import json, os, signal, sys, time
from pathlib import Path
root = Path(__file__).resolve().parents[3]
settings = json.loads((root / "fixture.json").read_text())
args = sys.argv[1:]
done = False
def stop(sig, frame):
    global done
    done = True
signal.signal(signal.SIGINT, stop)
(root / "probe-env.json").write_text(json.dumps({name:os.environ.get(name) for name in ("OMP_NUM_THREADS","OPENBLAS_NUM_THREADS")}))
(root / "probe-ready").write_text("ready")
deadline = time.monotonic() + 6
while settings.get("wait_probe") and not done and time.monotonic() < deadline:
    time.sleep(.01)
def value(option):
    return Path(args[args.index(option) + 1])
if not settings.get("no_ply"):
    value("--ply").write_text("ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n1 0 0\n0 1 0\n")
value("--trajectory").write_text("x,y,z\n0,0,0\n")
value("--session-report").write_text("frame=local metric\nimu_mode=gyro prediction and rotational deskew; stationary gravity initialization\n")
if "--map" in args:
    value("--map").mkdir()
    (value("--map") / "map.atlas").write_text("stub map")
sys.exit(settings.get("probe_exit", 0))
'''

ANALYZER = r'''
import hashlib, json, os, signal, sys, time
from pathlib import Path
root = Path(__file__).resolve().parents[2]
settings = json.loads((root / "fixture.json").read_text())
args = sys.argv[1:]
def value(option):
    return Path(args[args.index(option) + 1])
assert value("--session-metadata").is_file()
done = False
def stop(sig, frame):
    global done
    done = True
signal.signal(signal.SIGINT, stop)
(root / "analyzer-env.json").write_text(json.dumps({name:os.environ.get(name) for name in ("OMP_NUM_THREADS","OPENBLAS_NUM_THREADS")}))
(root / "analyzer-ready").write_text("ready")
deadline = time.monotonic() + 6
while settings.get("wait_analyzer") and not done and time.monotonic() < deadline:
    time.sleep(.01)
if done:
    sys.exit(130)
output = value("--output")
output.mkdir()
for name in ("scene.glb", "scene.json", "viewer.html"):
    (output / name).write_text("stub geometry artifact")
quality = {"safe_to_discard_source": settings.get("quality", True),
           "source_sha256": hashlib.sha256(value("--input").read_bytes()).hexdigest()}
if settings.get("missing_hash"):
    quality.pop("source_sha256")
if settings.get("wrong_hash"):
    quality["source_sha256"] = "0" * 64
(output / "quality.json").write_text(json.dumps(quality))
if settings.get("invalid_quality"):
    (output / "quality.json").write_text("invalid json")
if settings.get("mutate_source"):
    value("--input").write_text("changed after capture")
sys.exit(settings.get("analyzer_exit", 0))
'''


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="atlas session tests ")
        self.root = Path(self.temporary.name)
        probe = self.root / "build/mac-release/apps/atlas_livox_probe"
        probe.parent.mkdir(parents=True)
        probe.write_text("#!" + sys.executable + "\n" + PROBE)
        probe.chmod(0o755)
        config = self.root / "config/livox/mid360.json"
        config.parent.mkdir(parents=True)
        config.write_text("{}")
        analyzer = self.root / "tools/structure/analyze.py"
        analyzer.parent.mkdir(parents=True)
        analyzer.write_text(ANALYZER)
        geometry_python = self.root / "build/structure-venv/bin/python"
        geometry_python.parent.mkdir(parents=True)
        geometry_python.symlink_to(sys.executable)
        self.original = self.root / "output/existing/mapa.ply"
        self.original.parent.mkdir(parents=True)
        self.original.write_text("previous user measurements")
        self.process = None

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.communicate(timeout=2)
        self.temporary.cleanup()

    def start(self, settings=None, keep=False, threads=None):
        (self.root / "fixture.json").write_text(json.dumps(settings or {}))
        command = [sys.executable, str(RUNNER), "--root", str(self.root)]
        if keep:
            command.append("--keep-source")
        environment = os.environ.copy()
        for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
            environment.pop(name, None)
        if threads:
            environment.update(threads)
        self.process = subprocess.Popen(command, cwd="/tmp", stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, start_new_session=True,
                                        env=environment)

    def wait_marker(self, marker):
        deadline = time.monotonic() + 6
        while not (self.root / marker).exists() and time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.fail("Runner exited before marker: " + self.process.communicate()[0])
            time.sleep(.01)
        self.assertTrue((self.root / marker).exists(), marker)

    def finish(self):
        output, _ = self.process.communicate(timeout=15)
        statuses = list((self.root / "output").glob("*/session.json"))
        self.assertEqual(len(statuses), 1, output)
        self.assertEqual(self.original.read_text(), "previous user measurements")
        status = json.loads(statuses[0].read_text())
        return self.process.returncode, statuses[0].parent, status, output

    def test_sigint_finishes_capture_before_geometry_and_discards_only_new_source(self):
        self.start({"wait_probe": True})
        self.wait_marker("probe-ready")
        self.process.send_signal(signal.SIGINT)
        code, session, status, output = self.finish()
        self.assertEqual(code, 0, output)
        self.assertEqual(status["state"], "complete")
        self.assertTrue(status["source_deleted"])
        self.assertFalse((session / "mediciones.ply").exists())
        self.assertFalse((session / "mapa").exists())
        self.assertTrue((session / "geometria/scene.glb").is_file())
        self.assertIn("--session-report", status["capture_command"])

    def test_sigterm_uses_the_probes_graceful_sigint_handler(self):
        self.start({"wait_probe": True}, keep=True)
        self.wait_marker("probe-ready")
        self.process.terminate()
        code, session, status, output = self.finish()
        self.assertEqual(code, 0, output)
        self.assertFalse(status["source_deleted"])
        self.assertTrue((session / "mediciones.ply").is_file())
        self.assertTrue((session / "mapa/map.atlas").is_file())

    def test_failed_analyzer_retains_source_and_propagates_exit(self):
        self.start({"analyzer_exit": 7})
        code, session, status, output = self.finish()
        self.assertEqual(code, 7, output)
        self.assertFalse(status["source_deleted"])
        self.assertTrue((session / "mediciones.ply").is_file())

    def test_inadequate_quality_retains_source(self):
        self.start({"quality": False})
        code, session, status, output = self.finish()
        self.assertEqual(code, 2, output)
        self.assertEqual(status["state"], "needs_review")
        self.assertTrue((session / "mediciones.ply").is_file())

    def test_tracking_failure_is_not_hidden_by_successful_geometry(self):
        self.start({"probe_exit": 1})
        code, session, status, output = self.finish()
        self.assertEqual(code, 1, output)
        self.assertEqual(status["analyzer_exit"], 0)
        self.assertTrue((session / "mediciones.ply").is_file())

    def test_interrupted_reconstruction_keeps_measurements(self):
        self.start({"wait_analyzer": True})
        self.wait_marker("analyzer-ready")
        self.process.send_signal(signal.SIGINT)
        code, session, status, output = self.finish()
        self.assertEqual(code, 130, output)
        self.assertFalse(status["source_deleted"])
        self.assertTrue((session / "mediciones.ply").is_file())

    def test_missing_source_does_not_start_geometry(self):
        self.start({"no_ply": True})
        code, _, status, output = self.finish()
        self.assertEqual(code, 1, output)
        self.assertEqual(status["state"], "no_map")
        self.assertFalse((self.root / "analyzer-ready").exists())

    def test_source_changed_during_analysis_is_not_deleted(self):
        self.start({"mutate_source": True})
        code, session, _, output = self.finish()
        self.assertEqual(code, 2, output)
        self.assertEqual((session / "mediciones.ply").read_text(), "changed after capture")

    def test_malformed_quality_report_is_not_permission_to_delete(self):
        self.start({"invalid_quality": True})
        code, session, status, output = self.finish()
        self.assertEqual(code, 1, output)
        self.assertEqual(status["state"], "failed")
        self.assertTrue((session / "mediciones.ply").is_file())

    def test_missing_source_hash_in_quality_does_not_authorize_deletion(self):
        self.start({"missing_hash": True})
        code, session, status, output = self.finish()
        self.assertEqual(code, 2, output)
        self.assertFalse(status["source_deleted"])
        self.assertTrue((session / "mediciones.ply").is_file())

    def test_quality_for_a_different_source_does_not_authorize_deletion(self):
        self.start({"wrong_hash": True})
        code, session, status, output = self.finish()
        self.assertEqual(code, 2, output)
        self.assertFalse(status["source_deleted"])
        self.assertTrue((session / "mediciones.ply").is_file())

    def test_default_thread_limits_only_apply_to_analysis(self):
        self.start()
        code, _, _, output = self.finish()
        self.assertEqual(code, 0, output)
        self.assertEqual(json.loads((self.root / "probe-env.json").read_text()),
                         {"OMP_NUM_THREADS": None, "OPENBLAS_NUM_THREADS": None})
        self.assertEqual(json.loads((self.root / "analyzer-env.json").read_text()),
                         {"OMP_NUM_THREADS": "4", "OPENBLAS_NUM_THREADS": "1"})

    def test_explicit_thread_limits_are_preserved(self):
        expected = {"OMP_NUM_THREADS": "7", "OPENBLAS_NUM_THREADS": "3"}
        self.start(threads=expected)
        code, _, _, output = self.finish()
        self.assertEqual(code, 0, output)
        self.assertEqual(json.loads((self.root / "analyzer-env.json").read_text()), expected)


if __name__ == "__main__":
    unittest.main()
