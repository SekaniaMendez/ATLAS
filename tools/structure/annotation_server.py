#!/usr/bin/env python3
"""Local ATLAS annotation UI; optional authenticated LAN access for an iPad."""
from __future__ import annotations

import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
from pathlib import Path
import secrets
import socket
import sys
from urllib.parse import unquote, urlsplit

if __package__:
    from .annotation_store import AnnotationError, AnnotationStore
    from .category_registry import CategoryRegistry, CategoryRegistryError, BUILTIN_TAXONOMY, BUILTIN_CATALOG_ID
else:
    from annotation_store import AnnotationError, AnnotationStore
    from category_registry import CategoryRegistry, CategoryRegistryError, BUILTIN_TAXONOMY, BUILTIN_CATALOG_ID


MAX_BODY_BYTES = 1024 * 1024
STATIC_FILES = {
    "/": ("annotation.html", "text/html; charset=utf-8"),
    "/annotation.html": ("annotation.html", "text/html; charset=utf-8"),
    "/annotation.js": ("annotation.js", "application/javascript; charset=utf-8"),
    "/annotation_selection.js": ("annotation_selection.js", "application/javascript; charset=utf-8"),
    "/annotation.css": ("annotation.css", "text/css; charset=utf-8"),
}


def request_authorized(host, origin, port, *, writing=False, lan=False, local_hostnames=()):
    """Reject DNS rebinding and cross-origin writes, including Origin: null."""
    allowed_hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
    if port == 80:
        allowed_hosts |= {"127.0.0.1", "localhost"}
    if host not in allowed_hosts:
        if not lan or not isinstance(host, str):
            return False
        try:
            parsed = urlsplit("http://" + host)
            name = (parsed.hostname or "").lower().rstrip(".")
            if name in local_hostnames:
                private = True
            else:
                address = ipaddress.IPv4Address(name)
                private = any(address in ipaddress.ip_network(network) for network in
                              ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16", "169.254.0.0/16"))
            if (not private or parsed.port != port or parsed.username is not None
                    or parsed.password is not None or parsed.path or parsed.query or parsed.fragment
                    or host.lower() not in {f"{name}:{port}", f"{name}.:{port}"}):
                return False
        except (ValueError, TypeError):
            return False
    if writing and origin is not None and origin != f"http://{host}":
        return False
    return True


class AnnotationHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, stores, static_directory=None, access_token=None, registry=None):
        self.registry = registry
        self.stores = {store.dataset_id: store for store in stores}
        if len(self.stores) != len(stores):
            raise AnnotationError("Hay datasets con identificadores duplicados.")
        self.static_directory = Path(static_directory or Path(__file__).parent).resolve()
        self.lan_mode = address[0] == "0.0.0.0"
        self.access_token = access_token
        hostname = socket.gethostname().lower().rstrip(".")
        self.local_hostnames = {hostname if hostname.endswith(".local") else hostname + ".local"}
        super().__init__(address, AnnotationHandler)


class AnnotationHandler(BaseHTTPRequestHandler):
    server_version = "ATLASAnnotations/1"

    def log_message(self, fmt, *args):
        # Request paths can contain attacker-controlled strings; print only status
        # via the server startup and leave browser errors to its visible UI.
        pass

    def _response(self, status, data, content_type="application/json; charset=utf-8", filename=None):
        if not isinstance(data, bytes):
            data = json.dumps(data, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        if filename is not None:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(data)

    def _error(self, status, message, code="invalid_request", **fields):
        self._response(status, {"error": message, "code": code, **fields})

    def _guard(self, writing=False):
        hosts = self.headers.get_all("Host", [])
        origins = self.headers.get_all("Origin", [])
        if (len(hosts) != 1 or len(origins) > 1
                or not request_authorized(hosts[0], origins[0] if origins else None,
                                          self.server.server_port, writing=writing,
                                          lan=getattr(self.server, "lan_mode", False),
                                          local_hostnames=getattr(self.server, "local_hostnames", ()))):
            self._error(403, "Esta herramienta sólo acepta solicitudes locales del mismo origen.", "forbidden_origin")
            return False
        # A URL fragment never reaches the server. The UI exchanges its pairing
        # fragment for this header; access tokens are never accepted in queries.
        if getattr(self.server, "lan_mode", False):
            try:
                local_peer = ipaddress.ip_address(self.client_address[0]).is_loopback
                api_request = unquote(urlsplit(self.path).path, errors="strict").startswith("/api/")
            except (ValueError, UnicodeError):
                self._error(400, "Ruta o cliente inválido.")
                return False
            if api_request and not local_peer:
                authorization = self.headers.get_all("Authorization", [])
                expected = "Bearer " + self.server.access_token
                if (len(authorization) != 1
                        or not hmac.compare_digest(authorization[0].encode("utf-8"), expected.encode("utf-8"))):
                    self._error(401, "Abre el enlace de emparejamiento de ATLAS para acceder a las capturas.", "pairing_required")
                    return False
        return True

    def _path(self):
        parsed = urlsplit(self.path)
        if parsed.scheme or parsed.netloc or parsed.query or parsed.fragment:
            raise AnnotationError("Ruta inválida.")
        decoded = unquote(parsed.path, errors="strict")
        if "\\" in decoded or "\x00" in decoded or any(p in (".", "..") for p in decoded.split("/")):
            raise AnnotationError("Ruta inválida.")
        return decoded

    def _dataset(self, pieces):
        if len(pieces) < 3 or pieces[:2] != ["api", "datasets"]:
            raise FileNotFoundError()
        store = self.server.stores.get(pieces[2])
        if store is None:
            raise FileNotFoundError()
        return store

    def _dispatch_errors(self, callback):
        try:
            callback()
        except (AnnotationError, CategoryRegistryError) as error:
            fields = {"revision": error.revision} if hasattr(error, "revision") else {}
            self._error(error.status, str(error), error.code, **fields)
        except FileNotFoundError:
            self._error(404, "No se encontró el recurso solicitado.", "not_found")
        except (UnicodeError, ValueError, KeyError):
            self._error(400, "La solicitud o los datos no son válidos.")
        except OSError:
            self._error(500, "No se pudo leer o guardar el archivo local.", "storage_error")

    def do_GET(self):
        if self._guard():
            self._dispatch_errors(self._get)

    def _get(self):
        path = self._path()
        if path in STATIC_FILES:
            filename, mime = STATIC_FILES[path]
            self._response(200, (self.server.static_directory / filename).read_bytes(), mime)
            return
        if path == "/api/datasets":
            self._response(200, {"datasets": [store.summary() for store in self.server.stores.values()]})
            return
        if path == "/api/categories":
            registry = getattr(self.server, "registry", None)
            self._response(200, registry.read() if registry is not None else {
                "taxonomy": BUILTIN_TAXONOMY, "catalog_id": BUILTIN_CATALOG_ID, "revision": 0})
            return
        pieces = path.strip("/").split("/")
        store = self._dataset(pieces)
        if len(pieces) == 3:
            self._response(200, store.describe())
        elif pieces[3:] == ["points"]:
            store.read()  # Verify dataset integrity before displaying cached points.
            self._response(200, {"points": store.points.tolist(), "point_group_ids": store.point_group_ids.tolist(),
                                 "membership_kind": "immutable_base", "points_sha256": store.points_sha256})
        elif len(pieces) == 5 and pieces[3] == "export":
            filename = pieces[4]
            data = store.export_file(filename).read_bytes()
            mime = "application/octet-stream" if filename.endswith(".npz") else "application/json; charset=utf-8"
            self._response(200, data, mime, filename=filename)
        else:
            raise FileNotFoundError()

    def do_POST(self):
        if self._guard(writing=True):
            self._dispatch_errors(self._post)

    def _post(self):
        if self.headers.get_content_type() != "application/json":
            self._error(415, "Envía un cuerpo application/json.", "unsupported_media_type")
            return
        lengths = self.headers.get_all("Content-Length", [])
        if len(lengths) != 1 or self.headers.get("Transfer-Encoding") is not None:
            raise AnnotationError("Content-Length es obligatorio y debe ser único.")
        try:
            length = int(lengths[0])
        except ValueError as error:
            raise AnnotationError("Content-Length inválido.") from error
        if length < 1 or length > MAX_BODY_BYTES:
            self._error(413, "La solicitud excede el tamaño permitido.", "body_too_large")
            return
        self.connection.settimeout(10)
        raw = self.rfile.read(length)
        if len(raw) != length:
            raise AnnotationError("El cuerpo de la solicitud está incompleto.")
        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict):
            raise AnnotationError("La solicitud debe ser un objeto JSON.")
        path = self._path()
        if path == "/api/categories":
            if set(payload) != {"name"}:
                raise AnnotationError("La categoría requiere sólo su nombre.")
            registry = getattr(self.server, "registry", None)
            if registry is None:
                self._error(503, "Inicia el servidor con un catálogo compartido para crear categorías.", "category_catalog_disabled")
            else:
                self._response(200, registry.create(payload["name"]))
            return
        pieces = path.strip("/").split("/")
        store = self._dataset(pieces)
        if pieces[3:] == ["annotations"]:
            self._response(200, store.update(payload))
        elif pieces[3:] == ["point-groups"]:
            self._response(200, store.point_groups(payload))
        elif pieces[3:] == ["export"]:
            if set(payload) != {"revision"}:
                raise AnnotationError("La exportación requiere sólo la revisión actual.")
            self._response(200, store.export(payload["revision"]))
        else:
            raise FileNotFoundError()

    def do_OPTIONS(self):
        self._error(405, "No se permite acceso desde otro origen.", "method_not_allowed")


def create_server(dataset_dirs, port=8770, host="127.0.0.1", *, static_directory=None, access_token=None, taxonomy_path=None):
    if host not in ("127.0.0.1", "0.0.0.0"):
        raise AnnotationError("Usa 127.0.0.1 o activa explícitamente el acceso a la red local.")
    if host == "0.0.0.0" and (not isinstance(access_token, str)
                             or len(access_token) < 32 or not access_token.isascii()
                             or any(character.isspace() for character in access_token)):
        raise AnnotationError("El acceso LAN necesita un token de emparejamiento de al menos 32 caracteres.")
    if type(port) is not int or not 0 <= port <= 65535:
        raise AnnotationError("Puerto inválido.")
    registry = CategoryRegistry(taxonomy_path) if taxonomy_path is not None else None
    stores = [AnnotationStore(path, registry=registry) for path in dataset_dirs]
    if not stores:
        raise AnnotationError("Indica al menos un dataset.")
    return AnnotationHTTPServer((host, port), stores, static_directory=static_directory, access_token=access_token,
                                registry=registry)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", action="append", required=True, type=Path)
    parser.add_argument("--port", default=8770, type=int)
    parser.add_argument("--host", default="127.0.0.1", choices=["127.0.0.1"])
    parser.add_argument("--lan", action="store_true", help="Habilita acceso desde iPad en la red local con enlace de emparejamiento.")
    parser.add_argument("--taxonomy", type=Path, help="Ruta al catálogo compartido de categorías (por ejemplo output/categories.json).")
    args = parser.parse_args()
    try:
        token = secrets.token_urlsafe(32) if args.lan else None
        server = create_server(args.dataset, args.port, "0.0.0.0" if args.lan else args.host, access_token=token,
                               taxonomy_path=args.taxonomy)
        print(f"ATLAS: editor de etiquetas en http://127.0.0.1:{server.server_port}/", flush=True)
        if args.lan:
            print(f"iPad: http://IP_DE_LA_MAC:{server.server_port}/#token={token}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
    except (ValueError, OSError) as error:
        print(f"ATLAS: no se pudo abrir el editor: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
