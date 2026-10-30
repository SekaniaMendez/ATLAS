"""Read measured ATLAS point clouds without modifying the source capture."""
from pathlib import Path
import hashlib
import numpy as np


def load_cloud(path):
    path = Path(path)
    with path.open("rb") as stream:
        if stream.readline().strip() != b"ply":
            raise ValueError("La entrada debe ser un PLY de ATLAS.")
        count, properties, element, format_name = None, [], None, None
        for _ in range(256):
            line = stream.readline().decode("ascii").strip()
            if not line:
                raise ValueError("Cabecera PLY incompleta.")
            fields = line.split()
            if fields[0] == "format":
                format_name = fields[1]
            elif fields[0] == "element":
                element = fields[1]
                if element == "vertex":
                    count = int(fields[2])
                elif int(fields[2]) != 0:
                    raise ValueError("La entrada debe contener puntos, no caras u otros elementos.")
            elif fields[0] == "property" and element == "vertex":
                if fields[1] == "list":
                    raise ValueError("No se admiten propiedades variables en los vértices.")
                properties.append(fields[-1])
            elif line == "end_header":
                break
        else:
            raise ValueError("Cabecera PLY demasiado larga.")
        if format_name != "ascii" or count is None or not 3 <= count <= 2_000_000:
            raise ValueError("Se requiere PLY ASCII con entre 3 y 2.000.000 de puntos.")
        if not all(axis in properties for axis in ("x", "y", "z")):
            raise ValueError("Faltan coordenadas x/y/z en el PLY.")
        data = np.loadtxt(stream, dtype=np.float64, ndmin=2)
    if data.shape != (count, len(properties)):
        raise ValueError("El contenido PLY no coincide con la cantidad de vértices declarada.")
    points = data[:, [properties.index(axis) for axis in ("x", "y", "z")]]
    if not np.isfinite(points).all():
        raise ValueError("La captura contiene coordenadas no finitas.")
    # Stable ordering makes repeated extraction independent of voxel hash iteration.
    points = np.unique(points, axis=0)
    return points, {"source_name": path.name, "input_bytes": path.stat().st_size,
                    "input_points": count, "unique_points": len(points),
                    "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def load_session_metadata(path):
    if path is None:
        return {}, False
    values = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    gravity = (values.get("imu_mode", "").startswith("gyro prediction")
               and "stationary gravity initialization" in values.get("imu_mode", "")
               and int(values.get("scans_accepted", "0")) > 0)
    return values, gravity
