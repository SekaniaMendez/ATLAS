#!/usr/bin/env python3
"""Estimate phone→LiDAR time offset from recorded rotational motion, not arrival.

Inputs: full-rate ATLAS poses.jsonl and gyro.csv from the calibration exporter.
The reported uncertainty is empirical, not a bound on clock synchronization or
rolling-shutter exposure. This tool never changes a verified calibration flag.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_poses(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read schema 1 poses; require the recorder's complete pose-log marker."""
    path = path / "poses.jsonl" if path.is_dir() else path
    marker = json.loads((path.parent / "complete.json").read_text())
    if marker.get("complete") is not True or marker.get("pose_log_complete") is not True:
        raise ValueError("Camera pose log is incomplete or predates full-rate pose recording")
    stamps, matrices, normal = [], [], []
    with path.open() as stream:
        for sequence, line in enumerate(stream):
            if len(line) > 8192:
                raise ValueError("Oversized pose record")
            row = json.loads(line)
            stamp = row.get("timestamp_ns")
            if (row.get("schema_version") != 1 or row.get("sequence") != sequence
                    or type(stamp) is not int or not 0 < stamp < 2**64
                    or type(row.get("tracking_normal")) is not bool):
                raise ValueError("Invalid pose schema, sequence, tracking flag or timestamp")
            matrix = np.asarray(row.get("world_from_camera_row_major"), dtype=float)
            if matrix.shape != (16,) or not np.all(np.isfinite(matrix)):
                raise ValueError("Invalid pose matrix")
            matrix = matrix.reshape(4, 4)
            r = matrix[:3, :3]
            if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-4)
                    or not np.allclose(r.T @ r, np.eye(3), atol=1e-3)
                    or abs(np.linalg.det(r) - 1) > 1e-3):
                raise ValueError("Pose rotation is not a rigid proper rotation")
            stamps.append(stamp)
            matrices.append(r)
            normal.append(row["tracking_normal"])
    if marker.get("pose_samples") != len(stamps) or marker.get("pose_samples_dropped") != 0:
        raise ValueError("Pose completion counts do not match the log")
    return np.asarray(stamps, dtype=np.uint64), np.asarray(matrices), np.asarray(normal, dtype=bool)


def load_gyro(path: Path) -> tuple[np.ndarray, np.ndarray, int]:
    stamps, vectors, clocks = [], [], set()
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            stamp = int(row["timestamp_ns"])
            clock = int(row["clock_source"])
            if not 0 < stamp < 2**64 or clock not in (0, 1, 2):
                raise ValueError("Invalid or unknown gyro clock")
            stamps.append(stamp)
            clocks.add(clock)
            vectors.append([float(row[f"gyro_{axis}_rad_s"]) for axis in "xyz"])
    if len(clocks) != 1:
        raise ValueError("Gyro log must contain exactly one known clock source")
    return np.asarray(stamps, dtype=np.uint64), np.asarray(vectors), clocks.pop()


def _relative_seconds(stamps: np.ndarray) -> tuple[np.ndarray, int]:
    if len(stamps) < 2 or stamps.ndim != 1:
        raise ValueError("At least two timestamps are required")
    # Subtract integer anchors BEFORE floating conversion, including uint64 epochs.
    anchor = int(stamps[0])
    values = np.asarray([(int(t) - anchor) * 1e-9 for t in stamps])
    if np.any(np.diff(values) <= 0) or not np.all(np.isfinite(values)):
        raise ValueError("Timestamps must increase strictly; split reboot/clock-change sessions")
    return values, anchor


def _interpolate(t: np.ndarray, v: np.ndarray, query: np.ndarray,
                 maximum_gap_s: float) -> tuple[np.ndarray, np.ndarray]:
    right = np.searchsorted(t, query, side="right")
    inside = (right > 0) & (right < len(t))
    right = np.clip(right, 1, len(t) - 1)
    valid = inside & ((t[right] - t[right - 1]) <= maximum_gap_s)
    return np.interp(query, t, v), valid


def _correlation(a: np.ndarray, b: np.ndarray) -> float:
    a = a - a.mean()
    b = b - b.mean()
    scale = np.linalg.norm(a) * np.linalg.norm(b)
    return float(np.dot(a, b) / scale) if scale > 1e-12 else -1.0


def _excited(values: np.ndarray) -> bool:
    return (len(values) > 20 and np.std(values) >= 0.08
            and np.percentile(values, 90) - np.percentile(values, 10) >= 0.25)


def estimate_offset(phone_ns: np.ndarray, rotations: np.ndarray, tracking_normal: np.ndarray,
                    lidar_ns: np.ndarray, gyro_rad_s: np.ndarray, *, search_seconds: float = 15,
                    step_seconds: float = 0.002) -> dict:
    """Fit first 65% of the phone motion; reserve final 35% for validation.

    Search is relative to the two recording starts. Absolute offset is recovered
    from their integer timestamp anchors. Rotational speed magnitude is invariant
    to the unknown fixed camera↔LiDAR rotation, but repeated motion is ambiguous.
    """
    if not (np.isfinite(search_seconds) and 0 < search_seconds <= 120
            and np.isfinite(step_seconds) and 0.0005 <= step_seconds <= 0.02):
        raise ValueError("Search must be (0,120] s and grid step [0.5,20] ms")
    pt, p_anchor = _relative_seconds(phone_ns)
    lt, l_anchor = _relative_seconds(lidar_ns)
    if (rotations.shape != (len(pt), 3, 3) or tracking_normal.shape != (len(pt),)
            or gyro_rad_s.shape != (len(lt), 3)
            or not np.all(np.isfinite(rotations)) or not np.all(np.isfinite(gyro_rad_s))):
        raise ValueError("Invalid rotation/gyro samples")
    result = {
        "schema_version": 1, "method": "ARKit rotation-speed magnitude versus LiDAR gyro magnitude",
        "status": "rejected", "reasons": [], "clock_alignment_verified": False,
        "automatic_activation_allowed": False,
        "offset_definition": "lidar_timestamp_ns = phone_timestamp_ns + phone_to_lidar_offset_ns",
        "uncertainty_kind": "empirical_motion_fit_not_a_clock_error_bound",
        "search_relative_to_recording_starts_s": [-search_seconds, search_seconds],
        "grid_step_s": step_seconds,
        "limitations": ["No drift model; repeat for each session and validate over time",
                        "ARKit filtering and camera exposure/readout can bias this fit",
                        "A good motion fit does not certify extrinsics or a 15 ms clock bound"],
    }
    if pt[-1] < 12 or lt[-1] < 12:
        result["reasons"].append("insufficient_duration: record at least 12 s of varied rotation")
        return result
    delta = np.diff(pt)
    relative_r = np.swapaxes(rotations[:-1], 1, 2) @ rotations[1:]
    angle = np.arccos(np.clip((np.trace(relative_r, axis1=1, axis2=2) - 1) / 2, -1, 1))
    valid_pose = tracking_normal[:-1] & tracking_normal[1:] & (delta <= 0.05)
    mid = (pt[:-1] + pt[1:]) * 0.5
    speed = angle / delta
    valid_pose &= np.isfinite(speed) & (speed < 12)
    # No interpolation across tracking loss or a dropped-pose gap. 100 Hz is a
    # comparison grid, not a claim of 100 Hz independent camera observations.
    grid = np.arange(mid[0], mid[-1], 0.01)
    if valid_pose.sum() < 100:
        result["reasons"].append("insufficient_normal_tracking")
        return result
    phone_speed, phone_valid = _interpolate(mid, speed, grid, 0.05)
    neighbors = np.clip(np.searchsorted(mid, grid, side="right"), 1, len(mid) - 1)
    phone_valid &= valid_pose[neighbors - 1] & valid_pose[neighbors]
    split = pt[0] + (pt[-1] - pt[0]) * 0.65
    train = phone_valid & (grid < split)
    validation = phone_valid & (grid >= split)
    if train.sum() < 600 or validation.sum() < 300:
        result["reasons"].append("insufficient_tracking_coverage_in_training_or_reserved_segment")
        return result
    if not _excited(phone_speed[train]) or not _excited(phone_speed[validation]):
        result["reasons"].append("insufficient_excitation: vary rotation speed in both segments")
        return result
    gyro_speed = np.linalg.norm(gyro_rad_s, axis=1)
    if not _excited(gyro_speed):
        result["reasons"].append("insufficient_gyro_excitation")
        return result
    offsets = np.arange(-search_seconds, search_seconds + step_seconds * 0.5, step_seconds)

    def scores(mask: np.ndarray) -> np.ndarray:
        t, v = grid[mask], phone_speed[mask]
        values = np.full(len(offsets), -np.inf)
        for i, offset in enumerate(offsets):
            sampled, valid = _interpolate(lt, gyro_speed, t + offset, 0.03)
            if valid.sum() >= max(300, int(len(t) * 0.8)):
                values[i] = _correlation(v[valid], sampled[valid])
        return values

    train_scores = scores(train)
    validation_scores = scores(validation)
    if not np.any(np.isfinite(train_scores)) or not np.any(np.isfinite(validation_scores)):
        result["reasons"].append("insufficient_overlap: increase search or record both sensors together")
        return result
    best = int(np.argmax(train_scores))
    heldout_best = int(np.argmax(validation_scores))
    shift = float(offsets[best])
    distant = np.abs(offsets - shift) >= 0.15
    alternative = float(np.max(train_scores[distant])) if distant.any() else -np.inf
    margin = float(train_scores[best] - alternative)
    # Width of this local near-optimal peak, not a statistical confidence interval.
    left = right = best
    while left > 0 and train_scores[left - 1] >= train_scores[best] - 0.005:
        left -= 1
    while right + 1 < len(offsets) and train_scores[right + 1] >= train_scores[best] - 0.005:
        right += 1
    sampling_floor = max(float(np.median(delta)), float(np.median(np.diff(lt)))) * 0.5
    disagreement = abs(shift - float(offsets[heldout_best]))
    uncertainty = max((float(offsets[right]) - float(offsets[left])) * 0.5,
                      sampling_floor, disagreement, step_seconds)
    result.update(
        training_correlation=float(train_scores[best]),
        reserved_correlation_at_training_offset=(float(validation_scores[best])
                                                 if np.isfinite(validation_scores[best]) else None),
        reserved_best_correlation=float(validation_scores[heldout_best]),
        reserved_offset_disagreement_s=disagreement,
        alternative_peak_margin=margin if np.isfinite(margin) else None,
        empirical_uncertainty_ns=int(np.ceil(uncertainty * 1e9)),
        training_samples=int(train.sum()), reserved_samples=int(validation.sum()),
        reserved_segment_start_phone_ns=p_anchor + int(round(split * 1e9)),
        phone_sample_period_median_s=float(np.median(delta)),
        lidar_sample_period_median_s=float(np.median(np.diff(lt))),
    )
    if best in (0, len(offsets) - 1):
        result["reasons"].append("optimum_at_search_boundary")
    if train_scores[best] < 0.85 or validation_scores[best] < 0.8:
        result["reasons"].append("poor_match_in_training_or_reserved_segment")
    if margin < 0.03:
        result["reasons"].append("ambiguous_peak: repeated or slowly varying motion")
    if disagreement > max(0.02, step_seconds * 2):
        result["reasons"].append("reserved_segment_disagrees: drift, tracking error or different motion")
    if not result["reasons"]:
        result["status"] = "candidate_requires_review"
        result["phone_to_lidar_offset_ns"] = l_anchor - p_anchor + int(round(shift * 1e9))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", type=Path, required=True, help="Camera folder or poses.jsonl")
    parser.add_argument("--gyro", type=Path, required=True, help="Exporter gyro.csv")
    parser.add_argument("--output", type=Path, required=True, help="New JSON report (never overwritten)")
    parser.add_argument("--search-seconds", type=float, default=15)
    parser.add_argument("--step-ms", type=float, default=2)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists; choose a new report path")
    try:
        phone, rotations, normal = load_poses(args.poses)
        lidar, gyro, clock = load_gyro(args.gyro)
        report = estimate_offset(phone, rotations, normal, lidar, gyro,
                                 search_seconds=args.search_seconds, step_seconds=args.step_ms / 1000)
        report.update(lidar_clock_source=clock, pose_source=str(args.poses), gyro_source=str(args.gyro))
        with args.output.open("x") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0 if report["status"] == "candidate_requires_review" else 2
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.error(str(error))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
