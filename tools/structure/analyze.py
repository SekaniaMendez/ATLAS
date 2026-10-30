#!/usr/bin/env python3
"""Reconstruct a compact, bounded scene from an existing ATLAS PLY snapshot."""
import argparse
from dataclasses import asdict
from pathlib import Path
import sys
import time

from cloud_io import load_cloud, load_session_metadata
from plane_geometry import extract, derive_structural_frame
from structure_config import StructureConfig
from mesh_quality import measure_quality
from residual_geometry import reconstruct_residual
from scene_report import scene_description, retention_decision, write_json
from export_geometry import export_glb, write_viewer


def analyze(input_path, output_path, session_path=None, tolerance_m=.04, residual=True):
    started = time.monotonic()
    input_path, output_path = Path(input_path).resolve(), Path(output_path).resolve()
    if output_path.exists():
        raise ValueError("El directorio de salida ya existe; elige un destino nuevo.")
    if not .005 <= tolerance_m <= .10:
        raise ValueError("La tolerancia debe estar entre 0.005 y 0.10 metros.")
    points, metadata = load_cloud(input_path)
    if session_path is None:
        candidates = [input_path.parent / "slam_session.txt",
                      input_path.parent / "mapa" / "slam_session.txt",
                      input_path.parent / "map" / "slam_session.txt"]
        session_path = next((candidate for candidate in candidates if candidate.is_file()), None)
    session, gravity_aligned = load_session_metadata(session_path)
    metadata.update(slam_session=session, gravity_aligned=gravity_aligned,
                    orientation_note="Horizontal/vertical are relative to map Z; no object semantics are inferred.")
    config = StructureConfig()
    print(f"ATLAS: analizando {len(points):,} mediciones; extrayendo planos…", flush=True)
    patches = extract(points, config)
    planar_count = len(patches)
    print(f"ATLAS: {planar_count} planos con contorno medido. Reconstruyendo detalles…", flush=True)
    if residual:
        patches.extend(reconstruct_residual(points, patches, tolerance_m=tolerance_m))
    quality = measure_quality(points, patches, tolerance_m=tolerance_m)
    structural_frame = derive_structural_frame(patches[:planar_count], gravity_aligned=gravity_aligned)
    metadata.update(point_coverage=quality["point_coverage"], status="observed_surfaces",
                    planar_surfaces=planar_count, algorithm_config=asdict(config),
                    residual_enabled=residual, tolerance_m=tolerance_m,
                    coverage_tolerance_m=tolerance_m, model_filename="scene.glb")
    output_path.mkdir(parents=True, exist_ok=False)
    export_glb(output_path / "scene.glb", patches, metadata)
    description = scene_description(patches, dict(metadata), structural_frame)
    description["quality_file"] = "quality.json"
    description["quality_summary"] = {
        key: quality[key] for key in ("point_coverage", "unmodeled_points", "tolerance_m",
                                      "point_to_mesh_p95_m", "unsupported_area_fraction")}
    write_json(output_path / "scene.json", description)
    model_bytes = sum((output_path / name).stat().st_size for name in ("scene.glb", "scene.json"))
    ratio = metadata["input_bytes"] / model_bytes
    safe_to_discard, reasons = retention_decision(quality, metadata, ratio)
    quality.update(safe_to_discard_source=safe_to_discard, retention_reasons=reasons,
                   input_bytes=metadata["input_bytes"], model_bytes=model_bytes,
                   compression_ratio=ratio, source_sha256=metadata["source_sha256"],
                   planar_surfaces=planar_count, freeform_surfaces=len(patches) - planar_count,
                   structural_frame_available=structural_frame is not None,
                   elapsed_s=time.monotonic() - started)
    metadata.update(model_bytes=model_bytes, geometry_bytes=model_bytes, compression_ratio=ratio,
                    safe_to_discard_source=safe_to_discard, retention_reasons=reasons,
                    status="validated_approximation" if safe_to_discard else "partial_model")
    write_json(output_path / "quality.json", quality)
    write_viewer(output_path / "viewer.html", patches, metadata, structural_frame)
    # The portable viewer duplicates geometry for offline viewing. Report its
    # cost separately from the GLB + reference descriptions, including this JSON.
    bundle = ("scene.glb", "scene.json", "quality.json", "viewer.html")
    for _ in range(10):
        bundle_bytes = sum((output_path / name).stat().st_size for name in bundle)
        if quality.get("bundle_bytes") == bundle_bytes:
            break
        quality.update(bundle_bytes=bundle_bytes,
                       bundle_compression_ratio=metadata["input_bytes"] / bundle_bytes)
        write_json(output_path / "quality.json", quality)
    print(f"ATLAS: cobertura {quality['point_coverage']:.1%}, "
          f"{quality['triangles']:,} triángulos, modelo {model_bytes / 1024:.1f} KiB, "
          f"reducción {quality['compression_ratio']:.1f}×.", flush=True)
    print(f"Visor: {output_path / 'viewer.html'}", flush=True)
    if not quality["safe_to_discard_source"]:
        print("Modelo parcial: conserva las mediciones de respaldo. " + " ".join(reasons), flush=True)
    return quality


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--session-metadata", type=Path)
    parser.add_argument("--tolerance-m", type=float, default=.04)
    parser.add_argument("--no-residual", action="store_true", help="Sólo planos, para diagnóstico.")
    args = parser.parse_args()
    try:
        analyze(args.input, args.output, args.session_metadata, args.tolerance_m, not args.no_residual)
    except (ValueError, OSError, RuntimeError) as error:
        print("ATLAS: no se pudo completar la geometría: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
