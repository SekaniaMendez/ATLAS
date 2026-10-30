"""Category IDs stay stable across captures, processes, and catalog recovery."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "structure"))
from category_registry import BUILTIN_TAXONOMY, CategoryNameError, CategoryRegistry, CategoryRegistryError


class CategoryRegistryTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / "categories.json"
        self.registry = CategoryRegistry(self.path)

    def tearDown(self):
        self.temp.cleanup()

    def test_initial_ids_and_restart_are_stable(self):
        self.assertEqual(self.registry.read()["taxonomy"], BUILTIN_TAXONOMY)
        first = self.registry.create("Estante")
        self.assertTrue(first["created"])
        self.assertEqual(first["category"], {"id": 10, "key": "custom_000010", "name": "Estante"})
        restarted = CategoryRegistry(self.path)
        self.assertEqual(restarted.read()["catalog_id"], first["catalog_id"])
        self.assertEqual(restarted.create("Planta")["category"]["id"], 11)
        self.assertEqual(self.registry.read()["taxonomy"][:10], BUILTIN_TAXONOMY)

    def test_unicode_whitespace_casefold_and_builtin_aliases_reuse_categories(self):
        first = self.registry.create("  Café\u00a0  portátil  ")
        for name in ("CAFE\u0301 portátil", "Ｃａｆé\tportátil", "café PORTÁTIL"):
            result = self.registry.create(name)
            self.assertFalse(result["created"])
            self.assertEqual(result["category"], first["category"])
            self.assertEqual(result["revision"], 1)
        for name, index in ((" silla ", 3), ("CHAIR", 3), ("Sofá", 5), ("sofa", 5), ("suelo", 0)):
            result = self.registry.create(name)
            self.assertFalse(result["created"])
            self.assertEqual(result["category"]["id"], index)
        self.assertEqual(self.registry.read()["revision"], 1)

    def test_invalid_names_do_not_consume_ids(self):
        before = self.path.read_bytes()
        for name in (None, True, 12, [], {}, "", " \t\n ", "x" * 81, "armario\x00", "mesa\u200b", "\ud800"):
            with self.subTest(name=repr(name)), self.assertRaises(CategoryNameError):
                self.registry.create(name)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(self.registry.create("<Estante>")["category"]["id"], 10)

    def test_concurrent_instances_allocate_distinct_ids_and_reuse_duplicates(self):
        registries = [CategoryRegistry(self.path) for _ in range(8)]
        with ThreadPoolExecutor(max_workers=8) as pool:
            results = list(pool.map(lambda item: item[0].create(item[1]),
                                    zip(registries, ["Caja", "Lámpara", "Caja", "Planta", "CAJA", "Lámpara", "Banco", "  banco "])))
        self.assertEqual(sum(result["created"] for result in results), 4)
        categories = self.registry.read()["taxonomy"][10:]
        self.assertEqual({item["id"] for item in categories}, {10, 11, 12, 13})
        self.assertEqual(len({item["key"] for item in categories}), 4)
        self.assertEqual(len({result["category"]["id"] for result in results if result["category"]["name"].casefold() == "caja"}), 1)

    def test_concurrent_processes_share_one_id_namespace(self):
        module_dir = str(Path(__file__).resolve().parents[2] / "tools" / "structure")
        script = "import sys,json;sys.path.insert(0,sys.argv[1]);from category_registry import CategoryRegistry;print(json.dumps(CategoryRegistry(sys.argv[2]).create(sys.argv[3])))"
        processes = [subprocess.Popen([sys.executable, "-c", script, module_dir, str(self.path), name],
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                     for name in ("Maceta", "Mueble", "maceta", "Cortina")]
        results = []
        for process in processes:
            output, errors = process.communicate(timeout=15)
            self.assertEqual(process.returncode, 0, errors)
            results.append(json.loads(output))
        self.assertEqual(sum(item["created"] for item in results), 3)
        self.assertEqual({item["id"] for item in self.registry.read()["taxonomy"][10:]}, {10, 11, 12})

    def test_missing_registry_is_not_recreated_even_before_any_custom_label(self):
        self.path.unlink()
        with self.assertRaises(CategoryRegistryError):
            self.registry.read()
        with self.assertRaises(CategoryRegistryError):
            CategoryRegistry(self.path)
        self.assertFalse(self.path.exists())

    def test_catalog_truncation_replacement_and_remapping_fail_after_restart(self):
        self.registry.create("Estante")
        valid = self.path.read_text()
        malformed = []
        changed = json.loads(valid); changed["categories"][-1]["name"] = "Lámpara"; malformed.append(changed)
        changed = json.loads(valid); changed["categories"].pop(); changed["revision"] = 0; malformed.append(changed)
        changed = json.loads(valid); changed["catalog_id"] = "00000000-0000-0000-0000-000000000001"; malformed.append(changed)
        for changed in malformed:
            self.path.write_text(json.dumps(changed))
            with self.subTest(changed=changed), self.assertRaises(CategoryRegistryError):
                CategoryRegistry(self.path)
        self.path.write_text("{broken")
        with self.assertRaises(CategoryRegistryError):
            CategoryRegistry(self.path)

    def test_checkpoint_recovers_only_an_appended_prefix_after_interrupted_write(self):
        data = json.loads(self.path.read_text())
        data["categories"].append({"id": 10, "key": "custom_000010", "name": "Cojín"})
        data["revision"] = 1
        self.path.write_text(json.dumps(data))
        recovered = CategoryRegistry(self.path)
        self.assertEqual(recovered.read()["taxonomy"][-1]["name"], "Cojín")
        self.assertEqual(json.loads(recovered.identity_path.read_text())["category_count"], 11)

    def test_failed_write_and_mutated_response_cannot_change_registry(self):
        before = self.path.read_bytes()
        with patch("category_registry.os.replace", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.registry.create("Estante")
        self.assertEqual(self.path.read_bytes(), before)
        snapshot = self.registry.read()
        snapshot["taxonomy"][0]["id"] = 200
        self.assertEqual(self.registry.read()["taxonomy"], BUILTIN_TAXONOMY)
        self.assertEqual(list(self.path.parent.glob(".category-*.tmp")), [])

    def test_category_limit_rejects_allocation_before_int16_overflow(self):
        with patch("category_registry.MAX_CATEGORY_ID", 10):
            self.assertEqual(self.registry.create("Estante")["category"]["id"], 10)
            with self.assertRaises(CategoryNameError):
                self.registry.create("Maceta")
            self.assertEqual(self.registry.read()["revision"], 1)


if __name__ == "__main__":
    unittest.main()
