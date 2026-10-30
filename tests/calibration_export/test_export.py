#!/usr/bin/env python3
"""Run with exporter and make_fixture executables; uses temporary fixture directories."""
import csv
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def run(*args, success=True):
    result = subprocess.run([str(x) for x in args], capture_output=True, text=True)
    assert (result.returncode == 0) == success, (args, result.stdout, result.stderr)
    return result


def main():
    exporter, fixture = map(Path, sys.argv[1:])
    origin = 9_007_199_254_740_993
    with tempfile.TemporaryDirectory(prefix="atlas-calibration-export-") as work:
        work = Path(work)
        raw = work / "raw"
        run(fixture, raw, "normal")
        inspect = json.loads(run(exporter, "--input", raw, "--inspect").stdout)
        assert inspect["first_lidar_timestamp_ns"] == origin
        assert inspect["raw_points"] == 7 and inspect["selected_points"] == 4
        assert inspect["rejected_nonfinite"] == 1
        assert inspect["rejected_range"] == 1 and inspect["rejected_tag"] == 1
        assert inspect["coordinate_frame"] == "lidar_sensor"
        assert inspect["seconds"][0]["imu_samples"] == 2
        out = work / "selected"
        command = [exporter, "--input", raw, "--start-s", ".1", "--duration-s", ".4"]
        run(*command, "--output", out)
        meta = json.loads((out / "metadata.json").read_text())
        assert meta["selected_points"] == 1 and meta["selected_imu_samples"] == 1
        assert not meta["clock_alignment_verified"] and not meta["stationarity_verified"]
        assert meta["interval_end_ns_exclusive"] == origin + 500_000_000
        with (out / "points.csv").open() as src:
            points = list(csv.DictReader(src))
        assert len(points) == 1 and int(points[0]["timestamp_ns"]) == origin + 100_000_000
        assert points[0]["reflectivity"] == "99"
        with (out / "gyro.csv").open() as src:
            gyro = list(csv.DictReader(src))
        assert len(gyro) == 1 and int(gyro[0]["timestamp_ns"]) == origin + 100_000_000
        assert "element vertex 1\n" in (out / "cloud.ply").read_text()
        before = (out / "metadata.json").read_bytes()
        run(*command, "--output", out, success=False)
        assert (out / "metadata.json").read_bytes() == before
        kept = work / "keep_low"
        run(*command, "--output", kept, "--keep-low-confidence")
        assert json.loads((kept / "metadata.json").read_text())["selected_points"] == 2
        imu_only = work / "imu_only"
        run(*command, "--output", imu_only, "--imu-only")
        assert (imu_only / "gyro.csv").read_bytes() == (out / "gyro.csv").read_bytes()
        assert not (imu_only / "cloud.ply").exists() and not (imu_only / "points.csv").exists()
        assert not json.loads((imu_only / "metadata.json").read_text())["point_cloud_exported"]
        for mode in ("lidar_clock", "imu_clock", "regression"):
            invalid = work / mode
            run(fixture, invalid, mode)
            destination = work / (mode + "_out")
            run(exporter, "--input", invalid, "--output", destination, success=False)
            assert not destination.exists()
        truncated = work / "truncated"
        shutil.copytree(raw, truncated)
        path = truncated / "events.atlas"
        path.write_bytes(path.read_bytes()[:-4])
        run(exporter, "--input", truncated, "--inspect", success=False)
        run(exporter, "--input", raw, "--sensor-id", "999", "--inspect", success=False)
        for value in ("nan", "-1", "inf", "18446744074", "1junk"):
            run(exporter, "--input", raw, "--duration-s", value, "--inspect", success=False)
        run(exporter, "--input", raw, "--output", work / "empty", "--start-s", "50", success=False)
    print("Calibration export: interval boundaries, exact nanoseconds, filters, clock isolation, no overwrite and truncation passed")


if __name__ == "__main__":
    main()
