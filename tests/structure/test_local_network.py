import os
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "structure"))
from local_network import access_token, parse_interfaces


class LocalNetworkTest(unittest.TestCase):
    def test_hotspot_wifi_usb_and_ethernet_addresses_are_not_assumed_fixed(self):
        sample = """lo0: flags=8049<UP,LOOPBACK,RUNNING,MULTICAST> mtu 16384
    inet 127.0.0.1 netmask 0xff000000
en0: flags=8863<UP,BROADCAST,SMART,RUNNING,MULTICAST> mtu 1500
    inet 172.20.10.4 netmask 0xfffffff0
en8: flags=8863<UP,BROADCAST,RUNNING,MULTICAST> mtu 1500
    inet 192.168.1.1 netmask 0xffffff00
en9: flags=8863<UP,BROADCAST,RUNNING,MULTICAST> mtu 1500
    inet 169.254.22.1 netmask 0xffff0000
en10: flags=8863<BROADCAST,MULTICAST> mtu 1500
    inet 192.168.5.1 netmask 0xffffff00
"""
        result = parse_interfaces(sample)
        self.assertEqual([r["address"] for r in result], ["172.20.10.4", "192.168.1.1", "169.254.22.1"])

    def test_pairing_survives_restart_and_rejects_symlink(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "token"
            first = access_token(path)
            self.assertGreaterEqual(len(first), 32)
            self.assertEqual(access_token(path), first)
            self.assertEqual(os.stat(path).st_mode & 0o777, 0o600)
            other = Path(tmp) / "link"
            other.symlink_to(path)
            with self.assertRaises(ValueError):
                access_token(other)


if __name__ == "__main__":
    unittest.main()
