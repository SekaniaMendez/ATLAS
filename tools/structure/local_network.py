"""Local addresses and Bonjour advertisement for Wi-Fi, tethering and Ethernet."""
import ipaddress
import os
from pathlib import Path
import re
import secrets
import shutil
import subprocess


def parse_interfaces(text):
    addresses = []
    interface, active = None, False
    for line in text.splitlines():
        match = re.match(r"^(\w+):.*flags=.*<([^>]+)>", line)
        if match:
            interface, flags = match.groups()
            active = "UP" in flags.split(",") and "RUNNING" in flags.split(",")
        match = re.match(r"\s+inet (\d+\.\d+\.\d+\.\d+)\b", line)
        if not match or not active or not interface:
            continue
        address = ipaddress.IPv4Address(match.group(1))
        if address.is_private and not (address.is_loopback or address.is_unspecified or address.is_multicast):
            addresses.append({"interface": interface, "address": str(address),
                              "link_local": address.is_link_local})
    return sorted(addresses, key=lambda item: (item["link_local"], item["interface"], item["address"]))


def local_addresses():
    executable = shutil.which("ifconfig") or "/sbin/ifconfig"
    try:
        return parse_interfaces(subprocess.check_output([executable], text=True, stderr=subprocess.DEVNULL,
                                                        timeout=5))
    except (OSError, subprocess.SubprocessError):
        return []


def access_token(path):
    """Persist pairing across server restarts; secrets stay in ignored local build/."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ValueError("El archivo de emparejamiento no puede ser un enlace simbólico.")
    if path.exists():
        token = path.read_text(encoding="ascii").strip()
        if not re.fullmatch(r"[a-zA-Z0-9_-]{32,128}", token):
            raise ValueError("El archivo de emparejamiento no contiene un código válido.")
        return token
    token = secrets.token_urlsafe(32)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="ascii") as stream:
        stream.write(token + "\n")
    return token


def advertise(port):
    executable = shutil.which("dns-sd")
    if not executable:
        return None
    return subprocess.Popen([executable, "-R", "ATLAS Etiquetas", "_atlas-labels._tcp", "local", str(port)],
                            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_advertisement(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
