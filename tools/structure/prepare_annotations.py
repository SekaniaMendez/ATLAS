#!/usr/bin/env python3
"""Prepare immutable, measured groups for human annotation; never train or capture."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile
import time

import numpy as np

from cloud_io import load_cloud, load_session_metadata
from plane_geometry import extract, derive_structural_frame
from scene_report import write_json
from structure_config import StructureConfig
from semantic_groups import SemanticGroupConfig, build_groups


PIPELINE_VERSION = 1


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def session_file(input_path):
    return next((candidate for candidate in (
        input_path.parent / "slam_session.txt",
        input_path.parent / "mapa" / "slam_session.txt",
        input_path.parent / "map" / "slam_session.txt") if candidate.is_file()), None)


def signature(source_sha, session_sha, plane_config, group_config):
    content = dict(pipeline_version=PIPELINE_VERSION, source_sha256=source_sha,
                   session_sha256=session_sha, plane_config=plane_config,
                   group_config=group_config)
    return hashlib.sha256(json.dumps(content, sort_keys=True).encode()).hexdigest()


def prepare(input_path, output_path=None, *, reuse=False):
    """Build a dataset once, or reuse it only if source, metadata and config agree."""
    started = time.monotonic()
    input_path = Path(input_path).resolve()
    output_path = (Path(output_path).resolve() if output_path else
                   input_path.parent / "anotaciones")
    plane_config, group_config = StructureConfig(), SemanticGroupConfig()
    source_sha = sha256(input_path)
    session_path = session_file(input_path)
    session_sha = sha256(session_path) if session_path else None
    identity = signature(source_sha, session_sha, asdict(plane_config), asdict(group_config))
    if output_path.exists():
        if not reuse:
            raise ValueError("El destino ya existe. Usa uno nuevo para conservar sus etiquetas.")
        manifest = json.loads((output_path / "manifest.json").read_text(encoding="utf-8"))
        if (manifest.get("pipeline_signature") != identity or
                manifest.get("points_sha256") != sha256(output_path / "points.npz")):
            raise ValueError("La captura, configuración o puntos han cambiado. "
                             "Usa --destination con una carpeta nueva; se conservan las etiquetas anteriores.")
        print(f"ATLAS: reutilizando {output_path}; etiquetas conservadas.", flush=True)
        return output_path

    points, source = load_cloud(input_path)
    session, gravity = load_session_metadata(session_path)
    print(f"ATLAS: {input_path.parent.name} · {len(points):,} puntos. "
          "Extrayendo estructura y candidatos…", flush=True)
    patches = extract(points, plane_config)
    result = build_groups(points, patches, gravity_aligned=gravity, config=group_config)
    assignments = np.asarray(result["point_group_ids"], dtype=np.int32)
    groups = result["groups"]
    if (assignments.shape != (len(points),) or not groups or
            assignments.min() < 0 or assignments.max() >= len(groups) or
            not np.array_equal(np.bincount(assignments, minlength=len(groups)),
                               [group["point_count"] for group in groups])):
        raise ValueError("La agrupación no conserva exactamente todas las mediciones.")
    if source_sha != source["source_sha256"] or sha256(input_path) != source_sha:
        raise ValueError("La captura cambió durante la preparación; vuelve a intentarlo.")
    if session_path and sha256(session_path) != session_sha:
        raise ValueError("El informe de la captura cambió durante la preparación.")
    source.update(capture_id=input_path.parent.name, point_order="lexicographic_unique_xyz",
                  session_sha256=session_sha, slam_session=session, gravity_aligned=gravity)
    slug = re.sub(r"[^a-zA-Z0-9_-]", "-", input_path.parent.name)[:48] or "capture"
    manifest = dict(schema_version=1, pipeline_version=PIPELINE_VERSION,
                    pipeline_signature=identity, dataset_id=f"{slug}-{identity[:12]}",
                    generator="ATLAS geometric proposals + human annotation",
                    units="metres", map_frame="slam_map", map_up_axis="+Z",
                    point_count=len(points), input_features=["x", "y", "z"],
                    source=source, gravity_aligned=gravity,
                    groups=groups, diagnostics=result["diagnostics"],
                    plane_config=asdict(plane_config), group_config=asdict(group_config),
                    structural_frame=derive_structural_frame(patches, gravity_aligned=gravity),
                    planes=[dict(id=p["id"], orientation=p["kind"], origin_m=p["origin"],
                                 normal=p["normal"], area_m2=p["area_m2"],
                                 fit_rmse_m=p["rmse_m"]) for p in patches],
                    supervision="Only explicit human labels become training targets.",
                    grouping_note="Candidatos geométricos; un grupo puede contener partes o varios objetos.",
                    elapsed_s=time.monotonic() - started)
    # Stage the whole dataset before making it visible. No source is moved/deleted.
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".atlas-annotations-", dir=output_path.parent))
    try:
        np.savez_compressed(temporary / "points.npz", points=points.astype(np.float32),
                            point_group_ids=assignments)
        manifest["points_sha256"] = sha256(temporary / "points.npz")
        write_json(temporary / "manifest.json", manifest)
        if output_path.exists():
            raise ValueError("El destino apareció durante la preparación; no se reemplazará.")
        temporary.rename(output_path)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
    counts = {kind: sum(g["kind"] == kind for g in groups)
              for kind in ("structure", "object", "unassigned")}
    print(f"ATLAS: {counts['structure']} regiones estructurales, "
          f"{counts['object']} candidatos a objeto; {len(points):,} puntos conservados.\n"
          f"Datos para etiquetar: {output_path}", flush=True)
    return output_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", action="append", type=Path,
                        help="PLY existente. Se puede repetir; por defecto usa capturas de output/.")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--destination", type=Path,
                        help="Directorio nuevo alternativo; requiere una única entrada.")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--lan", action="store_true",
                        help="Permite iPhone/iPad en la red local con enlace de emparejamiento.")
    parser.add_argument("--token-file", type=Path,
                        help="Archivo local del código persistente (por defecto build/annotation-access-token).")
    parser.add_argument("--taxonomy", type=Path,
                        help="Catálogo compartido de categorías (por defecto output/categories.json).")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("El puerto debe estar entre 1 y 65535.")
    sources = args.input
    if not sources:
        sources = []
        for directory in sorted((args.root / "output").glob("*")):
            if not directory.is_dir():
                continue
            selected = next((directory / name for name in ("mapa.ply", "mediciones.ply", "map.ply")
                             if (directory / name).is_file()), None)
            if selected:
                sources.append(selected)
    if not sources:
        parser.error("No hay capturas PLY en output/. Usa --input archivo.ply.")
    if args.destination and len(sources) != 1:
        parser.error("--destination requiere una única captura con --input.")
    server = None
    advertisement = None
    try:
        datasets = [prepare(path, args.destination, reuse=True) for path in sources]
        if args.prepare_only:
            return 0
        from annotation_server import create_server
        from local_network import access_token, advertise, local_addresses
        token = access_token(args.token_file or args.root / "build" / "annotation-access-token") if args.lan else None
        server = create_server(datasets, port=args.port, host="0.0.0.0" if args.lan else "127.0.0.1",
                               access_token=token,
                               taxonomy_path=args.taxonomy or args.root / "output" / "categories.json")
        if args.lan:
            advertisement = advertise(args.port)
            print("Emparejar iPhone/iPad: usa la dirección de la Mac en su misma red.\n"
                  "Funciona también al conectar la Mac al internet compartido del iPhone.", flush=True)
            for address in local_addresses():
                print(f"  {address['interface']}: http://{address['address']}:{args.port}/#token={token}", flush=True)
        print(f"ATLAS · Etiquetado: http://127.0.0.1:{args.port}/\n"
              "Ctrl+C cierra el visor; las etiquetas guardadas permanecen en cada captura.", flush=True)
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        print("\nATLAS: visor cerrado. Capturas y etiquetas conservadas.", flush=True)
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        print("ATLAS: " + str(error), file=sys.stderr)
        return 1
    finally:
        if server is not None:
            server.server_close()
        if advertisement is not None:
            from local_network import stop_advertisement
            stop_advertisement(advertisement)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
