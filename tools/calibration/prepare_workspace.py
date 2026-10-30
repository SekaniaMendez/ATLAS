#!/usr/bin/env python3
"""Create a self-contained, local correspondence workspace; never calibrate implicitly.

Input: {camera_lens:'wide', views:[{id, split, lidar_directory, camera_frame}]}.
Paths are relative to the input manifest. camera_frame is the native JPEG's JSON.
All exported points are retained; crop the LiDAR export first for very large scenes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "structure"))
from cloud_io import load_cloud


def prepare(manifest_path, output):
    manifest_path, output = Path(manifest_path).resolve(), Path(output).resolve()
    spec = json.loads(manifest_path.read_text())
    if spec.get("camera_lens") != "wide":
        raise ValueError("Esta sesión requiere la lente wide / 1×.")
    entries = spec.get("views", [])
    if not entries or len(entries) > 32:
        raise ValueError("Se requieren entre 1 y 32 vistas.")
    prepared, seen = [], set()
    for entry in entries:
        key = entry["id"]
        if not isinstance(key, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", key) or key in seen:
            raise ValueError("Cada vista necesita un id único: letras, números, guion o guion bajo.")
        seen.add(key)
        if entry.get("split") not in ("train", "validation"):
            raise ValueError("split debe ser train o validation.")
        folder = (manifest_path.parent / entry["lidar_directory"]).resolve()
        lidar = json.loads((folder / "metadata.json").read_text())
        if lidar.get("coordinate_frame") != "lidar_sensor" or lidar.get("units") != "m":
            raise ValueError("Use una exportación LiDAR en metros y coordenadas del sensor, no un mapa SLAM.")
        frame_path = (manifest_path.parent / entry["camera_frame"]).resolve()
        frame = json.loads(frame_path.read_text())
        session = json.loads((frame_path.parent / "session.json").read_text())
        # Older ATLAS builds saved the selected physical lens's display label.
        if session.get("lens") not in ("wide", "1×"):
            raise ValueError("La captura de cámara no corresponde a la lente wide / 1×.")
        image = (frame_path.parent / frame["image"]).resolve()
        if image.parent != frame_path.parent or image.suffix.lower() not in (".jpg", ".jpeg"):
            raise ValueError("La imagen debe ser un JPEG de la misma carpeta de captura.")
        k = np.asarray(frame["intrinsics_row_major"], dtype=float)
        if k.shape != (9,):
            raise ValueError("intrinsics_row_major debe contener exactamente nueve valores.")
        k = k.reshape(3, 3)
        width, height = frame["width"], frame["height"]
        if any(type(n) is not int or n < 2 for n in (width, height)):
            raise ValueError("width y height deben ser enteros de al menos dos píxeles.")
        if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
                or not np.allclose(k[2], [0, 0, 1], atol=1e-9, rtol=0)
                or abs(k[0, 1]) > 1e-9 or abs(k[1, 0]) > 1e-9
                or not 0 <= k[0, 2] < width or not 0 <= k[1, 2] < height):
            raise ValueError("Intrínsecos o dimensiones inválidos.")
        timestamp = frame["timestamp_s"]
        if type(timestamp) not in (int, float) or not np.isfinite(timestamp) or timestamp < 0:
            raise ValueError("timestamp_s debe ser un tiempo de cámara finito y no negativo.")
        points, source = load_cloud(folder / "cloud.ply")
        if len(points) > 500_000:
            raise ValueError("Más de 500.000 puntos: exporte un intervalo más corto para seleccionar con precisión.")
        view = dict(id=key, split=entry["split"], image=f"{key}/image.jpg", width=width, height=height,
                    intrinsics_row_major=k.ravel().tolist(), correspondences=[],
                    cloud=f"{key}/points.f32", point_count=len(points),
                    image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
                    cloud_source=source, lidar_interval=lidar,
                    camera_timestamp_s=timestamp,
                    static_pair_confirmed=False)
        prepared.append((view, points, image))
    # Validate all inputs before creating a new output directory.
    output.mkdir(parents=True, exist_ok=False)
    for view, points, image in prepared:
        target = output / view["id"]
        target.mkdir()
        points.astype("<f4").tofile(target / "points.f32")
        shutil.copy2(image, target / "image.jpg")
    observations = dict(schema_version=1, camera_lens="wide", image_coordinates="native_unrotated",
                        views=[item[0] for item in prepared])
    (output / "observations.json").write_text(json.dumps(observations, indent=2, allow_nan=False))
    for name in ("collector.html", "collector.js"):
        shutil.copy2(Path(__file__).with_name(name), output / name)
    return observations


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = prepare(args.manifest, args.output)
        print(f"Preparadas {len(result['views'])} vistas en {args.output}. "
              "Sirva esa carpeta en localhost y abra collector.html.")
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f"Error: {error}\n")
