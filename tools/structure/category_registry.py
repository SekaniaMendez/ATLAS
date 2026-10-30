"""A shared append-only category namespace for human training labels."""
from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading
import unicodedata
import uuid


BUILTIN_TAXONOMY = [
    {"id": index, "key": key, "name": name}
    for index, (key, name) in enumerate((
        ("floor", "Suelo"), ("wall", "Pared"), ("ceiling", "Techo"),
        ("chair", "Silla"), ("table", "Mesa"), ("sofa", "Sofá"),
        ("cabinet", "Armario"), ("door", "Puerta"), ("window", "Ventana"),
        ("other", "Otro"),
    ))
]
BUILTIN_CATALOG_ID = "atlas-builtins-v1"
MAX_CATEGORY_ID = 32767  # -1 remains the unknown int16 training target.


class CategoryRegistryError(ValueError):
    status = 409
    code = "category_catalog_error"


class CategoryNameError(CategoryRegistryError):
    status = 400
    code = "invalid_category"


def normalize_name(value):
    if not isinstance(value, str):
        raise CategoryNameError("Escribe un nombre para la categoría.")
    text = unicodedata.normalize("NFKC", value)
    if any(unicodedata.category(char).startswith("C") and not char.isspace() for char in text):
        raise CategoryNameError("El nombre no admite caracteres de control u ocultos.")
    text = " ".join(text.split())
    if not 1 <= len(text) <= 80:
        raise CategoryNameError("El nombre de la categoría debe tener entre 1 y 80 caracteres.")
    return text


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def _fingerprint(categories):
    return hashlib.sha256(_json_bytes(categories)).hexdigest()


def _atomic_json(path, value):
    descriptor, temporary = tempfile.mkstemp(prefix=".category-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(_json_bytes(value))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CategoryRegistryError("No se puede leer el catálogo de categorías. Conserva y restaura sus archivos originales.") from error


class CategoryRegistry:
    """Allocate stable IDs under a process lock; never silently recreate a lost registry."""

    def __init__(self, path):
        candidate = Path(path).expanduser().absolute()
        if candidate.is_symlink():
            raise CategoryRegistryError("El catálogo no puede ser un enlace simbólico.")
        candidate.parent.mkdir(parents=True, exist_ok=True)
        self.path = candidate.parent.resolve() / candidate.name
        self.identity_path = self.path.with_name("." + self.path.name + ".identity")
        self.lock_path = self.path.with_name("." + self.path.name + ".lock")
        self._thread_lock = threading.RLock()
        self._catalog_id = None
        with self._locked():
            if not self.path.exists():
                if self.identity_path.exists():
                    raise CategoryRegistryError("Falta el catálogo compartido de categorías. Restaura el archivo original; no se reasignarán sus identificadores.")
                _atomic_json(self.path, {"schema_version": 1, "catalog_id": str(uuid.uuid4()),
                                        "revision": 0, "categories": deepcopy(BUILTIN_TAXONOMY)})
            self._load(initializing=True)

    @contextmanager
    def _locked(self):
        with self._thread_lock:
            for path in (self.path, self.identity_path, self.lock_path):
                if path.is_symlink():
                    raise CategoryRegistryError("Los archivos del catálogo no pueden ser enlaces simbólicos.")
            with self.lock_path.open("a+b") as stream:
                fcntl.flock(stream, fcntl.LOCK_EX)
                try:
                    yield
                finally:
                    fcntl.flock(stream, fcntl.LOCK_UN)

    def _validate(self, data):
        try:
            categories, catalog_id = data["categories"], data["catalog_id"]
            valid_id = str(uuid.UUID(catalog_id)) == catalog_id
            if (data["schema_version"] != 1 or not valid_id or not isinstance(categories, list)
                    or not len(BUILTIN_TAXONOMY) <= len(categories) <= MAX_CATEGORY_ID + 1
                    or type(data["revision"]) is not int
                    or data["revision"] != len(categories) - len(BUILTIN_TAXONOMY)
                    or categories[:len(BUILTIN_TAXONOMY)] != BUILTIN_TAXONOMY):
                raise ValueError()
            names = {item["key"].casefold() for item in BUILTIN_TAXONOMY}
            for index, category in enumerate(categories):
                name = normalize_name(category["name"])
                key = BUILTIN_TAXONOMY[index]["key"] if index < len(BUILTIN_TAXONOMY) else f"custom_{index:06d}"
                if (set(category) != {"id", "key", "name"} or type(category["id"]) is not int
                        or category["id"] != index or category["key"] != key or category["name"] != name
                        or name.casefold() in names):
                    raise ValueError()
                names.add(name.casefold())
        except (KeyError, TypeError, AttributeError, ValueError) as error:
            raise CategoryRegistryError("El catálogo de categorías está dañado o cambió sus identificadores. Restaura una copia válida.") from error

    def _load(self, initializing=False):
        data = _read_json(self.path)
        self._validate(data)
        catalog_id, categories = data["catalog_id"], data["categories"]
        if self._catalog_id is not None and self._catalog_id != catalog_id:
            raise CategoryRegistryError("El catálogo fue sustituido por otro. Los identificadores de entrenamiento no son intercambiables.")
        if self.identity_path.exists():
            identity = _read_json(self.identity_path)
            count = identity.get("category_count") if isinstance(identity, dict) else None
            if (not isinstance(identity, dict) or identity.get("schema_version") != 1
                    or identity.get("catalog_id") != catalog_id or type(count) is not int
                    or not len(BUILTIN_TAXONOMY) <= count <= len(categories)
                    or identity.get("prefix_sha256") != _fingerprint(categories[:count])):
                raise CategoryRegistryError("El catálogo perdió o modificó categorías existentes. Restaura el catálogo y su archivo de identidad juntos.")
        elif not initializing:
            raise CategoryRegistryError("Falta el archivo de identidad del catálogo. Restaura los archivos originales.")
        else:
            count = 0
        # The registry is written first. If interrupted before this checkpoint,
        # validate the complete old prefix and advance the checkpoint on reload.
        if count != len(categories):
            self._checkpoint(data)
        self._catalog_id = catalog_id
        return data

    def _checkpoint(self, data):
        _atomic_json(self.identity_path, {"schema_version": 1, "catalog_id": data["catalog_id"],
                                         "category_count": len(data["categories"]),
                                         "prefix_sha256": _fingerprint(data["categories"])})

    @staticmethod
    def _snapshot(data):
        return {"taxonomy": deepcopy(data["categories"]), "catalog_id": data["catalog_id"],
                "revision": data["revision"]}

    def read(self):
        with self._locked():
            return self._snapshot(self._load())

    def create(self, value):
        name = normalize_name(value)
        normalized = name.casefold()
        with self._locked():
            data = self._load()
            for category in data["categories"]:
                if (category["name"].casefold() == normalized
                        or category["id"] < len(BUILTIN_TAXONOMY) and category["key"].casefold() == normalized):
                    return {**self._snapshot(data), "category": deepcopy(category), "created": False}
            index = len(data["categories"])
            if index > MAX_CATEGORY_ID:
                raise CategoryNameError("El catálogo alcanzó el límite de 32768 categorías.")
            category = {"id": index, "key": f"custom_{index:06d}", "name": name}
            data["categories"].append(category)
            data["revision"] += 1
            _atomic_json(self.path, data)
            self._checkpoint(data)
            return {**self._snapshot(data), "category": deepcopy(category), "created": True}
