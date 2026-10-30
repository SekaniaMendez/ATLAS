"""API contract and local-origin checks without starting hardware or sockets."""
from email.message import Message
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from test_annotation_store import create_dataset
from annotation_store import AnnotationStore
from category_registry import CategoryRegistry
from annotation_server import AnnotationHandler, MAX_BODY_BYTES, create_server, request_authorized


class AnnotationAPITest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.store = AnnotationStore(create_dataset(Path(self.temp.name) / "dataset"))

    def tearDown(self):
        self.temp.cleanup()

    def request(self, method, path, body=None, host="127.0.0.1:8770", origin=None, mime="application/json", length=None,
                lan=False, peer="127.0.0.1", token=None):
        handler = AnnotationHandler.__new__(AnnotationHandler)
        handler.server = SimpleNamespace(stores={self.store.dataset_id: self.store}, server_port=8770,
                                         static_directory=Path(self.temp.name), lan_mode=lan, access_token="x" * 43,
                                         registry=getattr(self, "registry", None))
        handler.client_address = (peer, 49152)
        handler.path = path
        handler.headers = Message()
        handler.headers["Host"] = host
        if origin is not None:
            handler.headers["Origin"] = origin
        if token is not None:
            handler.headers["Authorization"] = "Bearer " + token
        raw = json.dumps(body).encode() if body is not None else b""
        handler.headers["Content-Type"] = mime
        handler.headers["Content-Length"] = str(len(raw) if length is None else length)
        handler.rfile = io.BytesIO(raw)
        handler.connection = SimpleNamespace(settimeout=lambda _: None)
        responses = []
        handler._response = lambda status, payload, content_type=None, filename=None: responses.append((status, payload, filename))
        getattr(handler, "do_" + method)()
        self.assertEqual(len(responses), 1)
        return responses[0]

    def test_api_edit_reload_export_and_download(self):
        status, body, _ = self.request("GET", "/api/datasets")
        self.assertEqual(status, 200)
        self.assertEqual(body["datasets"][0]["capture_id"], "capture-test")
        base = "/api/datasets/" + self.store.dataset_id
        status, body, _ = self.request("GET", base)
        self.assertEqual(body["taxonomy"][3]["key"], "chair")
        self.assertEqual(body["annotations"]["revision"], 0)
        status, body, _ = self.request("GET", base + "/points")
        self.assertEqual(len(body["points"]), 7)
        self.assertEqual(body["point_group_ids"], [0, 0, 1, 1, 2, 2, 3])
        status, body, _ = self.request("POST", base + "/annotations", {
            "revision": 0, "action": "label", "group_ids": ["object_a"], "label": "chair"},
            origin="http://127.0.0.1:8770")
        self.assertEqual(status, 200)
        self.assertEqual(body["revision"], 1)
        status, body, _ = self.request("POST", base + "/export", {"revision": 1})
        self.assertEqual(status, 200)
        self.assertEqual(body["summary"]["reviewed_points"], 2)
        npz = body["files"][0]
        status, body, name = self.request("GET", npz["url"])
        self.assertEqual(status, 200)
        self.assertEqual(name, "revision-000001.npz")
        self.assertTrue(body.startswith(b"PK"))

    def test_stale_revision_returns_conflict_for_ui(self):
        base = "/api/datasets/" + self.store.dataset_id
        payload = {"revision": 0, "action": "ignore", "group_ids": ["object_a"]}
        self.assertEqual(self.request("POST", base + "/annotations", payload)[0], 200)
        status, body, _ = self.request("POST", base + "/annotations", payload)
        self.assertEqual(status, 409)
        self.assertEqual(body["code"], "revision_conflict")
        self.assertEqual(body["revision"], 1)

    def test_create_point_group_returns_atomic_partition_and_export(self):
        base = "/api/datasets/" + self.store.dataset_id
        status, initial, _ = self.request("GET", base)
        self.assertEqual(initial["point_group_ids"], [0, 0, 1, 1, 2, 2, 3])
        payload = {"revision": 0, "point_indices": [0, 2, 3], "label": "chair"}
        status, created, _ = self.request("POST", base + "/point-groups", payload)
        self.assertEqual(status, 200)
        self.assertEqual(created["annotations"]["revision"], 1)
        self.assertEqual(created["manifest"]["partition_revision"], 1)
        self.assertEqual(created["point_group_ids"], [3, 0, 3, 3, 1, 1, 2])
        self.assertEqual(len(created["manifest"]["groups"]), 4)
        self.assertNotIn("object_a", created["annotations"]["groups"])
        self.assertEqual(self.request("GET", base)[1], created)
        # Canonical coordinates and their immutable base memberships can load independently.
        points = self.request("GET", base + "/points")[1]
        self.assertEqual(points["point_group_ids"], [0, 0, 1, 1, 2, 2, 3])
        self.assertEqual(points["membership_kind"], "immutable_base")
        self.assertEqual(self.request("POST", base + "/point-groups", payload)[0], 409)
        exported = self.request("POST", base + "/export", {"revision": 1})[1]
        self.assertEqual(exported["summary"]["reviewed_points"], 3)
        status, restored, _ = self.request("POST", base + "/point-groups", {
            "revision": 1, "action": "remove", "group_id": "manual-000001"})
        self.assertEqual(status, 200)
        self.assertEqual(restored["point_group_ids"], initial["point_group_ids"])
        self.assertEqual(self.request("POST", base + "/point-groups", {
            "revision": 2, "point_indices": [True], "label": "chair"})[0], 400)
        self.assertEqual(self.request("GET", exported["files"][0]["url"])[0], 200)

    def test_point_edits_keep_same_origin_and_pairing_requirements(self):
        base = "/api/datasets/" + self.store.dataset_id + "/point-groups"
        payload = {"revision": 0, "point_indices": [0], "label": "chair"}
        options = {"lan": True, "peer": "172.20.10.3", "host": "172.20.10.2:8770"}
        self.assertEqual(self.request("POST", base, payload, **options)[0], 401)
        self.assertEqual(self.request("POST", base, payload, token="x" * 43,
                                      origin="http://evil.example", **options)[0], 403)
        self.assertEqual(self.store.read()["revision"], 0)

    def test_categories_create_and_reuse_without_labeling_any_points(self):
        self.registry = CategoryRegistry(Path(self.temp.name) / "categories.json")
        self.store = AnnotationStore(self.store.directory, registry=self.registry)
        before = self.store.annotations_path.read_bytes()
        status, listed, _ = self.request("GET", "/api/categories")
        self.assertEqual(status, 200)
        self.assertEqual(len(listed["taxonomy"]), 10)
        status, created, _ = self.request("POST", "/api/categories", {"name": "Estante"})
        self.assertEqual(status, 200)
        self.assertTrue(created["created"])
        self.assertEqual(created["category"]["id"], 10)
        self.assertEqual(created["catalog_id"], listed["catalog_id"])
        self.assertEqual(self.store.annotations_path.read_bytes(), before)
        self.assertEqual(self.request("POST", "/api/categories", {"name": "  ESTANTE  "})[1]["created"], False)
        self.assertEqual(self.request("GET", "/api/categories")[1]["revision"], 1)
        base = "/api/datasets/" + self.store.dataset_id
        described = self.request("GET", base)[1]
        self.assertEqual(described["annotations"]["revision"], 0)
        self.assertEqual(described["taxonomy"][-1], created["category"])
        status, labeled, _ = self.request("POST", base + "/annotations", {
            "revision": 0, "group_ids": ["object_a"], "action": "label", "label": created["category"]["key"]})
        self.assertEqual(status, 200)
        self.assertEqual(labeled["groups"]["object_a"]["label"], "custom_000010")
        for payload in ({}, {"name": " "}, {"name": 42}, {"name": "Caja", "id": 999}):
            self.assertEqual(self.request("POST", "/api/categories", payload)[0], 400)

    def test_categories_require_configured_registry_pairing_and_same_origin(self):
        self.assertEqual(self.request("GET", "/api/categories")[0], 200)
        self.assertEqual(self.request("POST", "/api/categories", {"name": "Caja"})[0], 503)
        self.registry = CategoryRegistry(Path(self.temp.name) / "categories.json")
        options = {"lan": True, "peer": "172.20.10.3", "host": "172.20.10.2:8770"}
        self.assertEqual(self.request("GET", "/api/categories", **options)[0], 401)
        self.assertEqual(self.request("POST", "/api/categories", {"name": "Caja"}, **options)[0], 401)
        self.assertEqual(self.request("POST", "/api/categories", {"name": "Caja"}, token="x" * 43,
                                      origin="http://evil.example", **options)[0], 403)
        self.registry.path.unlink()
        status, error, _ = self.request("GET", "/api/categories")
        self.assertEqual(status, 409)
        self.assertEqual(error["code"], "category_catalog_error")

    def test_only_own_bonjour_hostname_is_allowed_in_lan_mode(self):
        options = dict(lan=True, local_hostnames={"atlas-mac.local"}, writing=True)
        self.assertTrue(request_authorized("atlas-mac.local:8770", "http://atlas-mac.local:8770", 8770, **options))
        self.assertFalse(request_authorized("other.local:8770", "http://other.local:8770", 8770, **options))
        self.assertFalse(request_authorized("atlas-mac.local:8770", "http://other.local:8770", 8770, **options))

    def test_external_origin_and_rebinding_cannot_write(self):
        base = "/api/datasets/" + self.store.dataset_id + "/annotations"
        payload = {"revision": 0, "action": "ignore", "group_ids": ["object_a"]}
        for origin in ("null", "https://example.com", "http://127.0.0.1:9000"):
            self.assertEqual(self.request("POST", base, payload, origin=origin)[0], 403)
        self.assertEqual(self.request("POST", base, payload, host="evil.example:8770")[0], 403)
        self.assertEqual(self.store.read()["revision"], 0)
        self.assertFalse(request_authorized("localhost:8770", "http://127.0.0.1:8770", 8770, writing=True))

    def test_body_limits_wrong_type_and_paths(self):
        base = "/api/datasets/" + self.store.dataset_id
        self.assertEqual(self.request("POST", base + "/annotations", {}, mime="text/plain")[0], 415)
        self.assertEqual(self.request("POST", base + "/annotations", {}, length=MAX_BODY_BYTES + 1)[0], 413)
        self.assertEqual(self.request("POST", base + "/annotations", [1, 2])[0], 400)
        for path in ("/../manifest.json", "/%2e%2e/manifest.json", base + "/export/../annotations.json"):
            self.assertEqual(self.request("GET", path)[0], 400)
        self.assertEqual(self.request("GET", "/api/datasets/missing")[0], 404)
        self.assertEqual(self.request("GET", "/manifest.json")[0], 404)
        self.assertEqual(self.request("OPTIONS", "/")[0], 405)

    def test_server_refuses_lan_binding(self):
        with self.assertRaises(ValueError):
            create_server([self.store.directory], host="0.0.0.0")

    def test_lan_api_requires_pairing_token_and_private_host(self):
        options = {"lan": True, "peer": "172.20.10.3", "host": "172.20.10.2:8770"}
        self.assertEqual(self.request("GET", "/api/datasets", **options)[0], 401)
        self.assertEqual(self.request("GET", "/%61pi/datasets", **options)[0], 401)
        self.assertEqual(self.request("GET", "/api/datasets", token="wrong", **options)[0], 401)
        self.assertEqual(self.request("GET", "/api/datasets", token="x" * 43, **options)[0], 200)
        options["host"] = "127.0.0.1:8770"
        self.assertEqual(self.request("GET", "/api/datasets", **options)[0], 401,
                         "A forged loopback Host cannot bypass peer authentication")
        for host in ("evil.example:8770", "8.8.8.8:8770", "172.20.10.2:8770@evil.example", "172.20.10.2:8770/"):
            self.assertFalse(request_authorized(host, None, 8770, lan=True))
        self.assertTrue(request_authorized("172.20.10.2:8770", "http://172.20.10.2:8770", 8770, writing=True, lan=True))
        self.assertFalse(request_authorized("172.20.10.2:8770", "http://192.168.0.3:8770", 8770, writing=True, lan=True))

    def test_lan_local_peer_needs_no_token(self):
        self.assertEqual(self.request("GET", "/api/datasets", lan=True)[0], 200)


if __name__ == "__main__":
    unittest.main()
