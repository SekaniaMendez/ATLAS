"""Human-reviewed labels over immutable, measured ATLAS point groups.

Automatic structural suggestions never become training targets until a person
confirms them. Point order and map coordinates remain those of points.npz.
"""
from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import threading

import numpy as np

if __package__:
    from .category_registry import BUILTIN_TAXONOMY, BUILTIN_CATALOG_ID
else:
    from category_registry import BUILTIN_TAXONOMY, BUILTIN_CATALOG_ID


TAXONOMY = deepcopy(BUILTIN_TAXONOMY)
LABEL_IDS = {item["key"]: item["id"] for item in TAXONOMY}
SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}\Z")
SHA256 = re.compile(r"[0-9a-f]{64}\Z")
EXPORT_NAME = re.compile(r"revision-([0-9]{6,12})\.(npz|json)\Z")
MANUAL_GROUP_ID = re.compile(r"manual-([0-9]{6,12})\Z")
MAX_SELECTED_POINTS = 100_000


class AnnotationError(ValueError):
    status = 400
    code = "invalid_request"


class RevisionConflict(AnnotationError):
    status = 409
    code = "revision_conflict"

    def __init__(self, revision):
        super().__init__("Las etiquetas cambiaron en otra pestaña. Recarga antes de guardar.")
        self.revision = revision


class DatasetChanged(AnnotationError):
    status = 409
    code = "dataset_changed"


def _now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _hash(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False,
                       sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def _reject_constant(value):
    raise AnnotationError("El JSON contiene un número no finito: " + value)


def _read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=_reject_constant)
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise AnnotationError("El archivo JSON no es válido.") from error


def _atomic_json(path, value):
    path = Path(path)
    descriptor, temporary = tempfile.mkstemp(prefix=".labels-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(_json_bytes(value))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _pending():
    return {"status": "pending", "label": None, "instance_id": None,
            "notes": "", "provenance": None, "updated_at": None}


class AnnotationStore:
    """One immutable capture, atomic label revisions, and reproducible exports."""

    def __init__(self, directory, registry=None):
        self.registry = registry
        self.directory = Path(directory).resolve(strict=True)
        if not self.directory.is_dir():
            raise AnnotationError("El dataset debe ser un directorio.")
        self._thread_lock = threading.RLock()
        self.manifest_path = self.directory / "manifest.json"
        self.points_path = self.directory / "points.npz"
        self.annotations_path = self.directory / "annotations.json"
        for path in (self.manifest_path, self.points_path, self.annotations_path,
                     self.directory / ".annotations.lock", self.directory / "training"):
            if path.is_symlink():
                raise AnnotationError("El dataset no admite enlaces simbólicos en sus archivos de trabajo.")
        self.manifest = _read_json(self.manifest_path)
        self._validate_manifest()
        self.dataset_id = self.manifest["dataset_id"]
        self.source_sha256 = self.manifest["source"]["source_sha256"]
        self.manifest_sha256 = _hash(self.manifest_path)
        self.points_sha256 = _hash(self.points_path)
        declared = self.manifest.get("points_sha256")
        if declared is not None and declared != self.points_sha256:
            raise DatasetChanged("Las mediciones no coinciden con el hash del manifiesto.")
        with np.load(self.points_path, allow_pickle=False) as payload:
            if "points" not in payload or "point_group_ids" not in payload:
                raise AnnotationError("Faltan puntos o grupos en points.npz.")
            points = payload["points"]
            memberships = payload["point_group_ids"]
        count = self.manifest["point_count"]
        if (points.shape != (count, 3) or not np.issubdtype(points.dtype, np.floating)
                or not np.isfinite(points).all()):
            raise AnnotationError("Los puntos deben ser coordenadas finitas Nx3.")
        if (memberships.shape != (count,) or not np.issubdtype(memberships.dtype, np.integer)
                or memberships.min() < 0 or memberships.max() >= len(self.manifest["groups"])):
            raise AnnotationError("Hay índices de grupo inválidos en points.npz.")
        counts = np.bincount(memberships.astype(np.int64), minlength=len(self.manifest["groups"]))
        if any(int(counts[index]) != group["point_count"]
               for index, group in enumerate(self.manifest["groups"])):
            raise AnnotationError("Los tamaños de los grupos no coinciden con las mediciones.")
        self.points = np.asarray(points, dtype=np.float32)
        if not np.isfinite(self.points).all():
            raise AnnotationError("Las coordenadas exceden el rango float32.")
        self.point_group_ids = memberships.astype(np.int32)
        self.points.flags.writeable = False
        self.point_group_ids.flags.writeable = False
        self.group_ids = [group["id"] for group in self.manifest["groups"]]
        with self._locked():
            if not self.annotations_path.exists():
                _atomic_json(self.annotations_path, {
                    "schema_version": 1, "dataset_id": self.dataset_id,
                    "source_sha256": self.source_sha256,
                    "manifest_sha256": self.manifest_sha256, "points_sha256": self.points_sha256,
                    "revision": 0, "next_instance_id": 1, "updated_at": None,
                    "groups": {group_id: _pending() for group_id in self.group_ids},
                })
            self._read_annotations()

    def _validate_manifest(self):
        manifest = self.manifest
        if (not isinstance(manifest, dict) or manifest.get("schema_version") != 1
                or not isinstance(manifest.get("dataset_id"), str)
                or not SAFE_ID.fullmatch(manifest["dataset_id"])):
            raise AnnotationError("El identificador o versión del dataset es inválido.")
        source = manifest.get("source")
        if (not isinstance(source, dict) or not isinstance(source.get("source_sha256"), str)
                or not SHA256.fullmatch(source["source_sha256"])):
            raise AnnotationError("Falta el SHA-256 de la captura original.")
        count = manifest.get("point_count")
        groups = manifest.get("groups")
        if (type(count) is not int or not 1 <= count <= 2_000_000
                or not isinstance(groups, list) or not 1 <= len(groups) <= count):
            raise AnnotationError("El manifiesto tiene cantidades inválidas.")
        ids = []
        for group in groups:
            if (not isinstance(group, dict) or not isinstance(group.get("id"), str)
                    or not SAFE_ID.fullmatch(group["id"])
                    or group.get("kind") not in ("structure", "object", "unassigned")
                    or type(group.get("point_count")) is not int or group["point_count"] < 1):
                raise AnnotationError("El manifiesto contiene un grupo inválido.")
            ids.append(group["id"])
        if len(ids) != len(set(ids)) or sum(g["point_count"] for g in groups) != count:
            raise AnnotationError("Los grupos no forman una partición de los puntos.")

    @contextmanager
    def _locked(self):
        # flock also protects against a second server using the same dataset.
        with self._thread_lock:
            lock_path = self.directory / ".annotations.lock"
            if lock_path.is_symlink():
                raise DatasetChanged("El archivo de bloqueo fue sustituido por un enlace.")
            with lock_path.open("a+b") as lock_file:
                fcntl.flock(lock_file, fcntl.LOCK_EX)
                try:
                    yield
                finally:
                    fcntl.flock(lock_file, fcntl.LOCK_UN)

    def _check_immutable(self):
        for path in (self.manifest_path, self.points_path, self.annotations_path):
            if path.is_symlink():
                raise DatasetChanged("Un archivo del dataset fue sustituido por un enlace.")
        if (_hash(self.manifest_path) != self.manifest_sha256
                or _hash(self.points_path) != self.points_sha256):
            raise DatasetChanged("La captura o sus grupos cambiaron; prepara un dataset nuevo.")

    def _catalog(self):
        if self.registry is not None:
            return self.registry.read()
        return {"taxonomy": deepcopy(TAXONOMY), "catalog_id": BUILTIN_CATALOG_ID, "revision": 0}

    def _bind_catalog(self, annotation, catalog):
        if self.registry is not None:
            annotation["taxonomy_catalog_id"] = catalog["catalog_id"]
            annotation["taxonomy_snapshot"] = deepcopy(catalog["taxonomy"])

    def _read_annotations(self, catalog=None):
        self._check_immutable()
        catalog = self._catalog() if catalog is None else catalog
        label_ids = {item["key"]: item["id"] for item in catalog["taxonomy"]}
        annotation = _read_json(self.annotations_path)
        if not isinstance(annotation, dict):
            raise AnnotationError("Las anotaciones no son válidas.")
        expected = {"schema_version": 1, "dataset_id": self.dataset_id,
                    "source_sha256": self.source_sha256, "manifest_sha256": self.manifest_sha256,
                    "points_sha256": self.points_sha256}
        if any(annotation.get(key) != value for key, value in expected.items()):
            raise DatasetChanged("Las anotaciones pertenecen a otra captura o a otros grupos.")
        bound = "taxonomy_catalog_id" in annotation or "taxonomy_snapshot" in annotation
        if bound:
            snapshot = annotation.get("taxonomy_snapshot")
            if (annotation.get("taxonomy_catalog_id") != catalog["catalog_id"]
                    or not isinstance(snapshot, list) or not len(TAXONOMY) <= len(snapshot) <= len(catalog["taxonomy"])
                    or snapshot != catalog["taxonomy"][:len(snapshot)]):
                raise DatasetChanged("Las etiquetas pertenecen a otro catálogo de categorías o sus identificadores cambiaron. Restaura el catálogo original.")
        revision = annotation.get("revision")
        next_id = annotation.get("next_instance_id")
        entries = annotation.get("groups")
        manual_groups = annotation.get("manual_groups", [])
        next_group = annotation.get("next_manual_group_id", 1)
        if (not isinstance(manual_groups, list) or len(manual_groups) > len(self.points)
                or type(next_group) is not int or not 1 <= next_group < 10**12):
            raise AnnotationError("Los grupos seleccionados manualmente no son válidos.")
        manual_ids, occupied = set(), np.zeros(len(self.points), dtype=bool)
        for group in manual_groups:
            if not isinstance(group, dict) or not isinstance(group.get("id"), str):
                raise AnnotationError("Hay un grupo manual inválido.")
            match = MANUAL_GROUP_ID.fullmatch(group["id"])
            if (not match or not 1 <= int(match.group(1)) < next_group
                    or group["id"] in self.group_ids or group["id"] in manual_ids
                    or group.get("provenance") != "human_selection"):
                raise AnnotationError("El identificador o procedencia de un grupo manual es inválido.")
            indices = self._point_indices(group.get("point_indices"))
            if indices != sorted(indices) or occupied[indices].any():
                raise AnnotationError("Los grupos manuales deben ser disjuntos y tener índices ordenados.")
            occupied[indices] = True
            manual_ids.add(group["id"])
        if (type(revision) is not int or not 0 <= revision < 10**12
                or type(next_id) is not int or not 1 <= next_id < 2**31
                or not isinstance(entries, dict) or set(entries) != set(self.group_ids) | manual_ids):
            raise AnnotationError("La revisión o las entradas de anotación son inválidas.")
        instance_labels = {}
        for entry in entries.values():
            if (not isinstance(entry, dict) or entry.get("status") not in ("pending", "reviewed", "ignored")
                    or not isinstance(entry.get("notes"), str) or len(entry["notes"]) > 500):
                raise AnnotationError("Hay una anotación inválida.")
            status = entry["status"]
            if status == "reviewed":
                label, instance = entry.get("label"), entry.get("instance_id")
                if (not isinstance(label, str) or label not in label_ids
                        or type(instance) is not int or not 1 <= instance < next_id
                        or entry.get("provenance") != "human"):
                    raise AnnotationError("Las etiquetas revisadas necesitan categoría e instancia humanas.")
                if label not in LABEL_IDS and (not bound or label not in {item["key"] for item in annotation["taxonomy_snapshot"]}):
                    raise DatasetChanged("La etiqueta personalizada no tiene una identidad de catálogo verificable.")
                if instance in instance_labels and instance_labels[instance] != label:
                    raise AnnotationError("Una misma instancia tiene categorías contradictorias.")
                instance_labels[instance] = label
            elif (entry.get("label") is not None or entry.get("instance_id") is not None
                  or entry.get("provenance") != ("human" if status == "ignored" else None)):
                raise AnnotationError("Los puntos pendientes o ignorados no deben tener categoría.")
        return annotation

    def read(self):
        with self._locked():
            return deepcopy(self._read_annotations())

    def _point_indices(self, values):
        if (not isinstance(values, list) or not 1 <= len(values) <= MAX_SELECTED_POINTS
                or any(type(value) is not int or not 0 <= value < len(self.points) for value in values)
                or len(set(values)) != len(values)):
            raise AnnotationError("Selecciona entre 1 y 100000 puntos medidos, con índices válidos sin repetir.")
        return values

    def _effective_partition(self, annotation):
        """Apply disjoint human selections without ever changing base files/order."""
        definitions = deepcopy(self.manifest["groups"])
        memberships = self.point_group_ids.copy()
        for group in annotation.get("manual_groups", []):
            memberships[group["point_indices"]] = len(definitions)
            definitions.append({"id": group["id"], "kind": "object", "manual": True,
                                "suggested_label": None, "confidence": None,
                                "reason": "Puntos medidos seleccionados manualmente.",
                                "surface_ids": []})
        counts = np.bincount(memberships, minlength=len(definitions))
        order = np.argsort(memberships, kind="stable")
        offsets = np.r_[0, np.cumsum(counts)]
        remap = np.full(len(definitions), -1, dtype=np.int32)
        groups = []
        for index, definition in enumerate(definitions):
            if not counts[index]:
                continue
            cloud = self.points[order[offsets[index]:offsets[index + 1]]].astype(np.float64)
            minimum, maximum = cloud.min(axis=0), cloud.max(axis=0)
            definition.update(point_count=int(counts[index]), bounds_m=[minimum.tolist(), maximum.tolist()],
                              centroid_m=cloud.mean(axis=0).tolist(), dimensions_m=(maximum - minimum).tolist())
            remap[index] = len(groups)
            groups.append(definition)
        return groups, remap[memberships]

    @staticmethod
    def _public_annotations(annotation, groups):
        result = {key: deepcopy(value) for key, value in annotation.items()
                  if key not in ("groups", "manual_groups", "next_manual_group_id")}
        result["groups"] = {group["id"]: deepcopy(annotation["groups"][group["id"]]) for group in groups}
        return result

    def _describe(self, annotation, catalog=None):
        catalog = self._catalog() if catalog is None else catalog
        groups, memberships = self._effective_partition(annotation)
        manifest = deepcopy(self.manifest)
        manifest["groups"] = groups
        manifest["partition_revision"] = annotation["revision"]
        return {"manifest": manifest, "annotations": self._public_annotations(annotation, groups),
                "taxonomy": deepcopy(catalog["taxonomy"]), "catalog_id": catalog["catalog_id"],
                "catalog_revision": catalog["revision"], "point_group_ids": memberships.tolist()}

    def describe(self):
        with self._locked():
            catalog = self._catalog()
            return self._describe(self._read_annotations(catalog), catalog)

    def summary(self):
        annotation = self.read()
        groups, _ = self._effective_partition(annotation)
        entries = self._public_annotations(annotation, groups)["groups"].values()
        return {"dataset_id": self.dataset_id,
                "capture_id": self.manifest["source"].get("capture_id", self.dataset_id),
                "source": deepcopy(self.manifest["source"]),
                "point_count": self.manifest["point_count"], "group_count": len(groups),
                "revision": annotation["revision"],
                "reviewed_groups": sum(e["status"] == "reviewed" for e in entries),
                "ignored_groups": sum(e["status"] == "ignored" for e in entries)}

    def update(self, request):
        if not isinstance(request, dict):
            raise AnnotationError("La solicitud debe ser un objeto JSON.")
        if set(request) - {"revision", "group_ids", "action", "label", "notes"}:
            raise AnnotationError("La solicitud contiene campos desconocidos.")
        revision = request.get("revision")
        if type(revision) is not int or revision < 0:
            raise AnnotationError("Debes enviar la revisión actual.")
        group_ids, action = request.get("group_ids"), request.get("action")
        if (not isinstance(group_ids, list) or not group_ids
                or not all(isinstance(item, str) for item in group_ids)
                or len(set(group_ids)) != len(group_ids)):
            raise AnnotationError("Selecciona grupos válidos sin repetirlos.")
        if action not in ("label", "ignore", "reset"):
            raise AnnotationError("Acción desconocida.")
        label, notes = request.get("label"), request.get("notes", "")
        if not isinstance(notes, str) or len(notes) > 500:
            raise AnnotationError("La nota admite hasta 500 caracteres.")
        if action == "label" and not isinstance(label, str):
            raise AnnotationError("Selecciona una categoría válida.")
        if action != "label" and label is not None:
            raise AnnotationError("Sólo la acción etiquetar admite una categoría.")
        with self._locked():
            catalog = self._catalog()
            annotation = self._read_annotations(catalog)
            if annotation["revision"] != revision:
                raise RevisionConflict(annotation["revision"])
            if action == "label" and label not in {item["key"] for item in catalog["taxonomy"]}:
                raise AnnotationError("Selecciona una categoría válida.")
            groups, _ = self._effective_partition(annotation)
            if not set(group_ids).issubset(group["id"] for group in groups):
                raise AnnotationError("Selecciona grupos existentes que aún tengan puntos.")
            if revision >= 10**12 - 1 or annotation["next_instance_id"] >= 2**31 - 1:
                raise AnnotationError("La captura alcanzó su límite de revisiones.")
            now = _now()
            instance_id = annotation["next_instance_id"] if action == "label" else None
            if action == "label":
                annotation["next_instance_id"] += 1
            for group_id in group_ids:
                annotation["groups"][group_id] = {
                    "status": {"label": "reviewed", "ignore": "ignored", "reset": "pending"}[action],
                    "label": label if action == "label" else None, "instance_id": instance_id,
                    "notes": notes.strip(), "provenance": None if action == "reset" else "human",
                    "updated_at": now,
                }
            annotation["revision"] += 1
            annotation["updated_at"] = now
            self._bind_catalog(annotation, catalog)
            _atomic_json(self.annotations_path, annotation)
            return self._public_annotations(annotation, groups)

    def point_groups(self, request):
        """Create an object from canonical point indices, or restore it to base groups."""
        if not isinstance(request, dict):
            raise AnnotationError("La solicitud debe ser un objeto JSON.")
        action = request.get("action", "create")
        allowed = ({"revision", "action", "point_indices", "label", "notes"} if action == "create"
                   else {"revision", "action", "group_id"} if action == "remove" else set())
        if not allowed or set(request) - allowed:
            raise AnnotationError("Acción de grupos de puntos o campos desconocidos.")
        revision = request.get("revision")
        if type(revision) is not int or revision < 0:
            raise AnnotationError("Debes enviar la revisión actual.")
        if action == "create":
            indices = sorted(self._point_indices(request.get("point_indices")))
            label, notes = request.get("label"), request.get("notes", "")
            if not isinstance(label, str):
                raise AnnotationError("Selecciona una categoría válida para el nuevo objeto.")
            if not isinstance(notes, str) or len(notes) > 500:
                raise AnnotationError("La nota admite hasta 500 caracteres.")
        elif not isinstance(request.get("group_id"), str):
            raise AnnotationError("Selecciona el identificador de un objeto creado manualmente.")
        with self._locked():
            catalog = self._catalog()
            annotation = self._read_annotations(catalog)
            if annotation["revision"] != revision:
                raise RevisionConflict(annotation["revision"])
            if action == "create" and label not in {item["key"] for item in catalog["taxonomy"]}:
                raise AnnotationError("Selecciona una categoría válida para el nuevo objeto.")
            if revision >= 10**12 - 1 or annotation["next_instance_id"] >= 2**31 - 1:
                raise AnnotationError("La captura alcanzó su límite de revisiones.")
            manual = annotation.get("manual_groups", [])
            now = _now()
            if action == "remove":
                group_id = request["group_id"]
                if group_id not in {group["id"] for group in manual}:
                    raise AnnotationError("Sólo se pueden deshacer objetos creados mediante selección de puntos.")
                manual = [group for group in manual if group["id"] != group_id]
                del annotation["groups"][group_id]
            else:
                selected = set(indices)
                remaining = []
                for group in manual:
                    group["point_indices"] = [index for index in group["point_indices"] if index not in selected]
                    if group["point_indices"]:
                        remaining.append(group)
                    else:
                        del annotation["groups"][group["id"]]
                next_group = annotation.get("next_manual_group_id", 1)
                group_id = f"manual-{next_group:06d}"
                while group_id in annotation["groups"]:
                    next_group += 1
                    group_id = f"manual-{next_group:06d}"
                if next_group >= 10**12 - 1:
                    raise AnnotationError("La captura alcanzó su límite de objetos manuales.")
                annotation["next_manual_group_id"] = next_group + 1
                manual = remaining + [{"id": group_id, "point_indices": indices,
                                       "created_at": now, "provenance": "human_selection"}]
                annotation["groups"][group_id] = {
                    "status": "reviewed", "label": label, "instance_id": annotation["next_instance_id"],
                    "notes": notes.strip(), "provenance": "human", "updated_at": now,
                }
                annotation["next_instance_id"] += 1
            annotation["manual_groups"] = manual
            annotation["revision"] += 1
            annotation["updated_at"] = now
            self._bind_catalog(annotation, catalog)
            description = self._describe(annotation, catalog)
            _atomic_json(self.annotations_path, annotation)
            return description

    def _training_arrays(self, annotation, groups, memberships, taxonomy):
        label_ids = {item["key"]: item["id"] for item in taxonomy}
        semantic = np.full(len(self.points), -1, dtype=np.int16)
        instances = np.full(len(self.points), -1, dtype=np.int32)
        reviewed = np.zeros(len(self.points), dtype=bool)
        ignored = np.zeros(len(self.points), dtype=bool)
        for index, group in enumerate(groups):
            entry = annotation["groups"][group["id"]]
            selected = memberships == index
            if entry["status"] == "reviewed":
                semantic[selected] = label_ids[entry["label"]]
                instances[selected] = entry["instance_id"]
                reviewed[selected] = True
            elif entry["status"] == "ignored":
                ignored[selected] = True
        return {"points": self.points, "point_group_ids": memberships,
                "base_point_group_ids": self.point_group_ids,
                "canonical_point_indices": np.arange(len(self.points), dtype=np.int32),
                "semantic_labels": semantic, "instance_ids": instances,
                "reviewed_mask": reviewed, "ignored_mask": ignored}

    def _export_result(self, revision, metadata):
        base = f"revision-{revision:06d}"
        return {"files": [{"name": f"{base}.{ext}",
                           "url": f"/api/datasets/{self.dataset_id}/export/{base}.{ext}"}
                          for ext in ("npz", "json")], "summary": metadata["summary"]}

    def export(self, revision):
        if type(revision) is not int or revision < 0:
            raise AnnotationError("Debes enviar la revisión actual para exportar.")
        with self._locked():
            catalog = self._catalog()
            annotation = self._read_annotations(catalog)
            if revision != annotation["revision"]:
                raise RevisionConflict(annotation["revision"])
            training = self.directory / "training"
            if training.is_symlink():
                raise DatasetChanged("El destino de entrenamiento fue sustituido por un enlace.")
            training.mkdir(exist_ok=True)
            destination = training / f"revision-{revision:06d}"
            annotation_hash = hashlib.sha256(_json_bytes(annotation)).hexdigest()
            if destination.exists():
                if (destination.is_symlink() or not destination.is_dir()
                        or (destination / "metadata.json").is_symlink()):
                    raise DatasetChanged("El destino de exportación no es válido.")
                metadata = _read_json(destination / "metadata.json")
                if (metadata.get("annotations_sha256") != annotation_hash
                        or metadata.get("points_sha256") != self.points_sha256
                        or (destination / "data.npz").is_symlink()
                        or metadata.get("npz_sha256") != _hash(destination / "data.npz")):
                    raise DatasetChanged("Ya existe una exportación incompatible o modificada de esta revisión.")
                return self._export_result(revision, metadata)
            groups, memberships = self._effective_partition(annotation)
            arrays = self._training_arrays(annotation, groups, memberships, catalog["taxonomy"])
            reviewed = arrays["reviewed_mask"]
            ignored = arrays["ignored_mask"]
            summary = {"dataset_id": self.dataset_id, "revision": revision,
                       "point_count": len(self.points), "reviewed_points": int(reviewed.sum()),
                       "ignored_points": int(ignored.sum()),
                       "pending_points": int((~reviewed & ~ignored).sum()),
                       "instances": int(len(np.unique(arrays["instance_ids"][reviewed]))),
                       "classes": {item["key"]: int(np.count_nonzero(arrays["semantic_labels"] == item["id"]))
                                   for item in catalog["taxonomy"]}, "split": "unassigned"}
            metadata = {
                "schema_version": 1, "dataset_id": self.dataset_id, "revision": revision,
                "created_at": _now(), "source": deepcopy(self.manifest["source"]),
                "source_sha256": self.source_sha256, "points_sha256": self.points_sha256,
                "manifest_sha256": self.manifest_sha256, "annotations_sha256": annotation_hash,
                "units": self.manifest.get("units", "metres"),
                "map_frame": self.manifest.get("map_frame", "slam_map"),
                "coordinate_policy": "Measured coordinates unchanged; no normalization or mesh vertices.",
                "point_order": self.manifest["source"].get("point_order", "points.npz_order"),
                "taxonomy": deepcopy(catalog["taxonomy"]), "unlabeled_value": -1,
                "taxonomy_catalog_id": catalog["catalog_id"], "taxonomy_revision": catalog["revision"],
                "label_provenance": "human_review_only", "automatic_suggestions_are_targets": False,
                "split": "unassigned",
                "split_group": self.manifest["source"].get("capture_id", self.dataset_id),
                "location_id": None, "requires_location_id_before_train_validation_split": True,
                "split_note": "Agrupa recorridos del mismo lugar antes de dividir entrenamiento y validación.",
                "groups": [{"group_index": i, **group, **annotation["groups"][group["id"]]}
                           for i, group in enumerate(groups)],
                "base_group_ids": self.group_ids,
                "manual_groups": deepcopy(annotation.get("manual_groups", [])),
                "point_group_policy": "Effective partition after human selections; base membership retained separately.",
                "summary": summary,
            }
            temporary = Path(tempfile.mkdtemp(prefix=".export-", dir=training))
            try:
                data_path = temporary / "data.npz"
                with data_path.open("wb") as stream:
                    np.savez_compressed(stream, **arrays)
                    stream.flush()
                    os.fsync(stream.fileno())
                metadata["npz_sha256"] = _hash(data_path)
                _atomic_json(temporary / "metadata.json", metadata)
                # Readers see the complete pair only after this directory rename.
                os.rename(temporary, destination)
            finally:
                if temporary.exists():
                    shutil.rmtree(temporary)
            return self._export_result(revision, metadata)

    def export_file(self, filename):
        match = EXPORT_NAME.fullmatch(filename)
        if not match:
            raise AnnotationError("Nombre de exportación inválido.")
        revision = int(match.group(1))
        if filename != f"revision-{revision:06d}.{match.group(2)}":
            raise AnnotationError("Nombre de exportación inválido.")
        with self._locked():
            self._read_annotations()
            directory = self.directory / "training" / f"revision-{revision:06d}"
            path = directory / ("data.npz" if match.group(2) == "npz" else "metadata.json")
            if (directory.parent.is_symlink() or directory.is_symlink() or path.is_symlink()
                    or (directory / "metadata.json").is_symlink() or not path.is_file()):
                raise FileNotFoundError("La exportación solicitada no existe.")
            metadata = _read_json(directory / "metadata.json")
            if (metadata.get("dataset_id") != self.dataset_id
                    or metadata.get("revision") != revision
                    or metadata.get("points_sha256") != self.points_sha256):
                raise DatasetChanged("La exportación no corresponde a esta captura.")
            if match.group(2) == "npz" and metadata.get("npz_sha256") != _hash(path):
                raise DatasetChanged("Los datos exportados cambiaron después de guardarlos.")
            return path
