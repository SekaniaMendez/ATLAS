#!/usr/bin/env python3
"""Fit a proposed LiDAR -> ARKit camera transform from measured 3D/2D pairs.

Only training views participate in fitting or outlier selection. Validation
views are held out completely. A passing report is never a verified profile.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class CalibrationError(ValueError):
    """The input cannot support a meaningful extrinsic calibration."""


@dataclass(frozen=True)
class Policy:
    # Engineering screening thresholds, not a guarantee of physical accuracy.
    ransac_px: float = 4.0
    min_inlier_fraction: float = 0.65
    max_validation_median_px: float = 3.0
    max_validation_p95_px: float = 5.0
    max_validation_error_px: float = 10.0
    max_translation_m: float = 1.0


@dataclass(frozen=True)
class View:
    id: str
    split: str
    width: int
    height: int
    k: np.ndarray
    xyz: np.ndarray
    uv: np.ndarray
    pair_ids: tuple[str, ...]


def _array(value, shape, name):
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise CalibrationError(f"{name}: se requieren números finitos") from error
    if array.shape != shape or not np.isfinite(array).all():
        raise CalibrationError(f"{name}: forma esperada {shape}, sólo números finitos")
    return array


def _spectrum(points):
    return np.linalg.svd(points - points.mean(axis=0), compute_uv=False)


def parse_observations(document):
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise CalibrationError("schema_version debe ser 1")
    if document.get("image_coordinates") != "native_unrotated":
        raise CalibrationError("Las observaciones deben usar imagen nativa sin rotar")
    if document.get("camera_lens") not in ("wide", "ultraWide"):
        raise CalibrationError("camera_lens debe ser wide o ultraWide")
    raw_views = document.get("views")
    if not isinstance(raw_views, list):
        raise CalibrationError("Falta la lista views")
    views, seen_ids = [], set()
    for raw in raw_views:
        if not isinstance(raw, dict):
            raise CalibrationError("Cada vista debe ser un objeto")
        view_id, split = raw.get("id"), raw.get("split")
        if not isinstance(view_id, str) or not view_id.strip() or view_id in seen_ids:
            raise CalibrationError("Cada vista necesita un id único no vacío")
        seen_ids.add(view_id)
        if split not in ("train", "validation"):
            raise CalibrationError(f"{view_id}: split debe ser train o validation")
        if raw.get("static_pair_confirmed") is not True:
            raise CalibrationError(f"{view_id}: confirmar static_pair_confirmed=true sólo tras comprobar la toma inmóvil común")
        dimensions = (raw.get("width"), raw.get("height"))
        if any(type(n) is not int or n < 2 for n in dimensions):
            raise CalibrationError(f"{view_id}: width y height deben ser enteros positivos")
        width, height = dimensions
        k = _array(raw.get("intrinsics_row_major"), (9,), f"{view_id}.K").reshape(3, 3)
        if (not np.allclose(k[2], [0, 0, 1], atol=1e-9, rtol=0)
                or abs(k[0, 1]) > 1e-9 or abs(k[1, 0]) > 1e-9
                or k[0, 0] <= 0 or k[1, 1] <= 0
                or not 0 <= k[0, 2] < width or not 0 <= k[1, 2] < height):
            raise CalibrationError(f"{view_id}: intrínsecos pinhole ARKit inválidos")
        raw_pairs = raw.get("correspondences")
        if not isinstance(raw_pairs, list) or len(raw_pairs) < 4:
            raise CalibrationError(f"{view_id}: se requieren al menos 4 correspondencias")
        xyz, uv, pair_ids = [], [], []
        for index, pair in enumerate(raw_pairs):
            if not isinstance(pair, dict):
                raise CalibrationError(f"{view_id}: correspondencia inválida")
            xyz.append(_array(pair.get("xyz_lidar_m"), (3,), f"{view_id}.xyz"))
            uv.append(_array(pair.get("uv_px"), (2,), f"{view_id}.uv"))
            pair_id = pair.get("id", str(index))
            if not isinstance(pair_id, str) or not pair_id or pair_id in pair_ids:
                raise CalibrationError(f"{view_id}: id de correspondencia duplicado/inválido")
            pair_ids.append(pair_id)
        xyz, uv = np.asarray(xyz), np.asarray(uv)
        if not ((uv >= 0).all() and (uv[:, 0] < width).all() and (uv[:, 1] < height).all()):
            raise CalibrationError(f"{view_id}: píxeles fuera de la imagen nativa")
        if len(np.unique(xyz, axis=0)) != len(xyz) or len(np.unique(uv, axis=0)) != len(uv):
            raise CalibrationError(f"{view_id}: correspondencias duplicadas")
        sx, su = _spectrum(xyz), _spectrum(uv)
        if sx[0] < 0.05 or sx[1] / sx[0] < 0.01 or su[0] < 10 or su[1] / su[0] < 0.01:
            raise CalibrationError(f"{view_id}: puntos coincidentes, colineales o sin extensión suficiente")
        views.append(View(view_id, split, width, height, k, xyz, uv, tuple(pair_ids)))
    training = [v for v in views if v.split == "train"]
    validation = [v for v in views if v.split == "validation"]
    if len(training) < 3 or len(validation) < 2:
        raise CalibrationError("Se requieren al menos 3 vistas train y 2 validation independientes")
    # Planar individual boards are expected; their combined poses must explore 3D.
    spectrum = _spectrum(np.concatenate([v.xyz for v in training]))
    if spectrum[2] / spectrum[0] < 0.005:
        raise CalibrationError("Conjunto train degenerado: variar distancia/inclinación; no todo en un plano")
    # Catch accidental copy/paste of an entire pose into another split. This
    # cannot prove physical independence; that remains a capture requirement.
    for held_out in validation:
        for fit in training:
            if held_out.xyz.shape == fit.xyz.shape and np.allclose(held_out.xyz, fit.xyz, atol=1e-9, rtol=0):
                raise CalibrationError("Una vista validation repite puntos de una vista train")
    return views


def _stack(views):
    xyz = np.concatenate([v.xyz for v in views])
    uv = np.concatenate([v.uv for v in views])
    ks = np.concatenate([np.repeat(v.k[None, :, :], len(v.xyz), axis=0) for v in views])
    return xyz, uv, ks


def _project(parameters, xyz, ks):
    camera = xyz @ Rotation.from_rotvec(parameters[:3]).as_matrix().T + parameters[3:]
    # Finite penalty near/behind the camera keeps least-squares well-defined;
    # cheirality is independently checked before a proposal can pass screening.
    depth = camera[:, 2]
    projected = np.einsum("nij,nj->ni", ks, camera)
    safe_depth = np.where(np.abs(depth) < 1e-8, 1e-8, depth)
    return projected[:, :2] / safe_depth[:, None], depth


def _stats(errors):
    if len(errors) == 0:
        return {"count": 0, "median_px": None, "p95_px": None, "max_px": None, "rms_px": None}
    return {"count": int(len(errors)), "median_px": float(np.median(errors)),
            "p95_px": float(np.percentile(errors, 95)), "max_px": float(np.max(errors)),
            "rms_px": float(np.sqrt(np.mean(errors ** 2)))}


def solve_document(document, policy=Policy()):
    for value in (policy.ransac_px, policy.max_validation_median_px,
                  policy.max_validation_p95_px, policy.max_validation_error_px,
                  policy.max_translation_m):
        if not np.isfinite(value) or value <= 0:
            raise CalibrationError("Los umbrales de calidad deben ser positivos y finitos")
    if not 0 < policy.min_inlier_fraction <= 1:
        raise CalibrationError("min_inlier_fraction debe pertenecer a (0,1]")
    views = parse_observations(document)
    training = [v for v in views if v.split == "train"]
    held_out = [v for v in views if v.split == "validation"]
    xyz, uv, ks = _stack(training)
    normalized = (uv - ks[:, :2, 2]) / np.stack([ks[:, 0, 0], ks[:, 1, 1]], axis=1)
    # Each view has its own K. A common largest focal length makes the normalized
    # RANSAC tolerance conservative relative to the requested pixel threshold.
    norm_threshold = policy.ransac_px / float(max(ks[:, 0, 0].max(), ks[:, 1, 1].max()))
    cv2.setRNGSeed(17)
    ok, rvec, tvec, initial_inliers = cv2.solvePnPRansac(
        np.ascontiguousarray(xyz), np.ascontiguousarray(normalized), np.eye(3), None,
        iterationsCount=3000, reprojectionError=norm_threshold, confidence=0.999,
        flags=cv2.SOLVEPNP_EPNP)
    if not ok or initial_inliers is None or len(initial_inliers) < 6:
        raise CalibrationError("PnP no encontró un consenso suficiente de correspondencias")
    parameters = np.concatenate([rvec.ravel(), tvec.ravel()])
    mask = np.zeros(len(xyz), dtype=bool)
    mask[initial_inliers.ravel()] = True
    fit = None
    # Refit pixel residuals with each frame's K. Only training pairs can affect
    # the estimate or consensus; no held-out residual is used to pick a model.
    for _ in range(4):
        if np.count_nonzero(mask) < 6:
            raise CalibrationError("Quedaron menos de 6 correspondencias coherentes")
        fit = least_squares(
            lambda p: (_project(p, xyz[mask], ks[mask])[0] - uv[mask]).ravel(),
            parameters, method="lm", max_nfev=1000, ftol=1e-11, xtol=1e-11, gtol=1e-11)
        parameters = fit.x
        errors = np.linalg.norm(_project(parameters, xyz, ks)[0] - uv, axis=1)
        updated = errors <= policy.ransac_px
        if np.array_equal(mask, updated):
            break
        mask = updated
    # The final optimizer must match the final consensus used in the report.
    if np.count_nonzero(mask) < 6:
        raise CalibrationError("Consenso final insuficiente")
    fit = least_squares(
        lambda p: (_project(p, xyz[mask], ks[mask])[0] - uv[mask]).ravel(),
        parameters, method="lm", max_nfev=1000, ftol=1e-11, xtol=1e-11, gtol=1e-11)
    parameters = fit.x
    if not np.isfinite(parameters).all():
        raise CalibrationError("El ajuste produjo parámetros no finitos")
    projected_train, train_depth = _project(parameters, xyz, ks)
    errors_train = np.linalg.norm(projected_train - uv, axis=1)
    validation_xyz, validation_uv, validation_ks = _stack(held_out)
    projected_validation, validation_depth = _project(parameters, validation_xyz, validation_ks)
    errors_validation = np.linalg.norm(projected_validation - validation_uv, axis=1)
    # Observability diagnostic after scaling translation columns by the scene
    # length, to avoid comparing derivatives in radians and metres directly.
    scene_scale = float(np.median(np.linalg.norm(xyz[mask] - xyz[mask].mean(axis=0), axis=1)))
    scaled_jacobian = fit.jac * np.array([1, 1, 1, scene_scale, scene_scale, scene_scale])
    singular = np.linalg.svd(scaled_jacobian, compute_uv=False)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else None
    rotation_cv = Rotation.from_rotvec(parameters[:3]).as_matrix()
    translation_cv = parameters[3:]
    camera_axes_to_arkit = np.diag([1.0, -1.0, -1.0])
    rotation_arkit = camera_axes_to_arkit @ rotation_cv
    translation_arkit = camera_axes_to_arkit @ translation_cv
    validation_stats = _stats(errors_validation)
    view_reports, offset = [], 0
    for view in views:
        p, depth = _project(parameters, view.xyz, np.repeat(view.k[None], len(view.xyz), axis=0))
        error = np.linalg.norm(p - view.uv, axis=1)
        if view.split == "train":
            selected = mask[offset:offset + len(view.xyz)]
            offset += len(view.xyz)
        else:
            selected = np.ones(len(view.xyz), dtype=bool)
        view_reports.append({"id": view.id, "split": view.split, "all_pairs": _stats(error),
                             "fit_inliers": _stats(error[selected]) if view.split == "train" else None,
                             "positive_depth_fraction": float(np.mean(depth > 0.05)),
                             "pairs": [{"id": pair_id, "error_px": float(e),
                                        "residual_uv_px": (predicted - measured).tolist(),
                                        "predicted_uv_px": predicted.tolist(),
                                        "used_for_fit": bool(use) if view.split == "train" else False}
                                       for pair_id, e, predicted, measured, use in zip(
                                           view.pair_ids, error, p, view.uv, selected)]})
    fit_views = [v for v in view_reports if v["split"] == "train"]
    inlier_spectrum = _spectrum(xyz[mask])
    checks = {
        "input_geometry": {"passed": True, "detail": "3+ train, 2+ held-out views, noncoplanar train geometry"},
        "optimizer_converged": {"passed": bool(fit.success), "value": str(fit.message)},
        "training_consensus": {"passed": bool(mask.mean() >= policy.min_inlier_fraction),
                               "value": float(mask.mean()), "minimum": policy.min_inlier_fraction},
        "training_inlier_error": {"passed": bool(np.percentile(errors_train[mask], 95) <= policy.ransac_px),
                                  "value_px": float(np.percentile(errors_train[mask], 95)), "maximum_px": policy.ransac_px},
        "consensus_spans_views": {"passed": all(v["fit_inliers"]["count"] >= 3 and
                                                    v["fit_inliers"]["count"] / v["all_pairs"]["count"] >= 0.5
                                                    for v in fit_views)},
        "consensus_nonplanar": {"passed": bool(inlier_spectrum[2] / inlier_spectrum[0] >= 0.005),
                                "value": float(inlier_spectrum[2] / inlier_spectrum[0]), "minimum": 0.005},
        "observability": {"passed": condition is not None and condition < 1e6,
                          "scaled_jacobian_condition": condition, "maximum": 1e6},
        "in_front_of_camera": {"passed": bool((train_depth > 0.05).all() and (validation_depth > 0.05).all())},
        "translation_bound": {"passed": bool(np.linalg.norm(translation_cv) <= policy.max_translation_m),
                              "value_m": float(np.linalg.norm(translation_cv)), "maximum_m": policy.max_translation_m},
        "held_out_median": {"passed": validation_stats["median_px"] <= policy.max_validation_median_px,
                            "value_px": validation_stats["median_px"], "maximum_px": policy.max_validation_median_px},
        "held_out_p95": {"passed": validation_stats["p95_px"] <= policy.max_validation_p95_px,
                         "value_px": validation_stats["p95_px"], "maximum_px": policy.max_validation_p95_px},
        "held_out_max": {"passed": validation_stats["max_px"] <= policy.max_validation_error_px,
                         "value_px": validation_stats["max_px"], "maximum_px": policy.max_validation_error_px},
    }
    passed = all(check["passed"] for check in checks.values())
    return {
        "schema_version": 1, "kind": "atlas_camera_lidar_extrinsic_proposal",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "status": "candidate_requires_review" if passed else "candidate_rejected_by_quality_gates",
        "verified": False, "quality_gates_passed": passed,
        "camera_lens": document["camera_lens"], "image_coordinates": "native_unrotated",
        "transform_convention": "p_camera_arkit = R * p_lidar_m + t; ARKit +X right, +Y up, -Z forward",
        "lidar_to_camera": {"rotation_matrix_row_major": rotation_arkit.ravel().tolist(),
                            "translation_m": translation_arkit.tolist()},
        "opencv_transform": {"rotation_matrix_row_major": rotation_cv.ravel().tolist(),
                             "translation_m": translation_cv.tolist()},
        "camera_center_lidar_m": (-rotation_arkit.T @ translation_arkit).tolist(),
        "training_all_pairs": _stats(errors_train), "training_fit_inliers": _stats(errors_train[mask]),
        "validation_all_pairs": validation_stats, "views": view_reports, "quality_gates": checks,
        "limitations": ["No automatic profile installation or verification.",
                        "No phone/LiDAR time alignment is estimated by this solver.",
                        "Static, independent capture poses and correct physical correspondences are required.",
                        "Pinhole model with the measured per-frame K; no distortion parameters are fitted.",
                        "Reprojection thresholds are engineering screens, not millimetre accuracy certification."],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("observations", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="Archivo NUEVO; no sobrescribe")
    parser.add_argument("--ransac-px", type=float, default=4.0)
    parser.add_argument("--max-validation-p95-px", type=float, default=5.0)
    parser.add_argument("--max-translation-m", type=float, default=1.0)
    args = parser.parse_args(argv)
    try:
        raw = args.observations.read_bytes()
        document = json.loads(raw)
        result = solve_document(document, Policy(ransac_px=args.ransac_px,
                                                max_validation_p95_px=args.max_validation_p95_px,
                                                max_translation_m=args.max_translation_m))
        result["source"] = {"path": str(args.observations.resolve()), "sha256": hashlib.sha256(raw).hexdigest()}
        # Exclusive creation keeps installed profiles and existing reports safe.
        encoded = json.dumps(result, indent=2, allow_nan=False) + "\n"
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
        stats = result["validation_all_pairs"]
        print(f"Propuesta NO VERIFICADA: {args.output}\n"
              f"Validación independiente: mediana {stats['median_px']:.3f}px, "
              f"p95 {stats['p95_px']:.3f}px, máximo {stats['max_px']:.3f}px")
        if not result["quality_gates_passed"]:
            print("No supera: " + ", ".join(k for k, v in result["quality_gates"].items() if not v["passed"]))
            return 2
        return 0
    except (CalibrationError, OSError, ValueError, cv2.error) as error:
        print(f"ATLAS calibración: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
