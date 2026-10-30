#!/bin/bash
# Start a SLAM session; Ctrl+C closes capture and exports compact geometry.
set -euo pipefail

usage() {
  cat <<'HELP'
Uso: ./startAtlas.sh [--keep-source] [--check | --help]

Sin opciones: comprueba la red local e inicia el SLAM del MID-360.
Al pulsar Ctrl+C crea geometría GLB, marcos JSON y un visor HTML en output/fecha-hora-XXXXXX/.
El PLY intermedio se elimina solo tras validar el modelo y si la captura terminó sin errores.
Si algo falla, conserva las mediciones para recuperar la sesión.
Mantén el equipo quieto hasta ver state=tracking. Espera a que termine la reconstrucción.

  --keep-source  Conserva también el PLY y el archivo de voxels (necesario para etiquetar).
  --check        Comprueba entorno, configuración, IP y puertos sin iniciar una sesión.
  --help         Muestra esta ayuda.

Preparar el entorno de geometría: ./scripts/setupGeometry.sh
HELP
}

ATLAS_CHECK_ONLY=false
ATLAS_KEEP_SOURCE=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) ATLAS_CHECK_ONLY=true ;;
    --keep-source) ATLAS_KEEP_SOURCE=true ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
  shift
done

ATLAS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null && pwd -P)"
ATLAS_PROBE="$ATLAS_ROOT/build/mac-release/apps/atlas_livox_probe"
ATLAS_CONFIG="$ATLAS_ROOT/config/livox/mid360.json"
ATLAS_GEOMETRY_PYTHON="$ATLAS_ROOT/build/structure-venv/bin/python"
cd -- "$ATLAS_ROOT"

if [[ ! -x "$ATLAS_PROBE" ]]; then
  printf 'Falta el ejecutable Release: %s\nCompílalo desde la carpeta del proyecto:\n' "$ATLAS_PROBE" >&2
  printf '%s\n' '  cmake --preset mac-release -DATLAS_ENABLE_LIVOX=ON -DATLAS_ENABLE_UI_STREAM=ON' \
    '  cmake --build --preset mac-release' >&2
  exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
  printf 'Se necesita python3 para comprobar la configuración de red.\n' >&2
  exit 1
fi
if [[ ! -x "$ATLAS_GEOMETRY_PYTHON" ]] || \
    ! "$ATLAS_GEOMETRY_PYTHON" -c 'import open3d, numpy, scipy, shapely' >/dev/null 2>&1; then
  printf '%s\n' 'Falta el entorno de geometría. Prepáralo antes de capturar:' \
    "  $ATLAS_ROOT/scripts/setupGeometry.sh" >&2
  exit 1
fi
if [[ ! -f "$ATLAS_ROOT/tools/structure/analyze.py" || ! -f "$ATLAS_ROOT/tools/structure/run_session.py" ]]; then
  printf 'Faltan los programas de reconstrucción en tools/structure/.\n' >&2
  exit 1
fi
if ! "$ATLAS_PROBE" --help | python3 -c 'import sys; sys.exit(0 if "--session-report" in sys.stdin.read() else 1)'; then
  printf '%s\n' 'El ejecutable necesita actualizarse para guardar los metadatos de geometría:' \
    '  cmake --build --preset mac-release' >&2
  exit 1
fi

# Read the configured address rather than assuming a fixed Ethernet interface.
# Binding temporary local sockets sends no commands to the sensor.
python3 - "$ATLAS_CONFIG" <<'PY'
import errno
import ipaddress
import json
import socket
import sys
sys.path.insert(0, "tools/structure")
from local_network import local_addresses

def stop(message):
    print("ATLAS: " + message, file=sys.stderr)
    sys.exit(1)

try:
    with open(sys.argv[1], encoding="utf-8") as source:
        config = json.load(source)
    host = config["MID360"]["host_net_info"]
    host_ip = host.get("host_ip", host.get("cmd_data_ip"))
    address = ipaddress.IPv4Address(host_ip)
    if address.is_unspecified or address.is_multicast:
        raise ValueError("se requiere una IPv4 local concreta para el MID-360")
    ports = {56000}
    for name in ("cmd_data_port", "push_msg_port", "point_data_port", "imu_data_port"):
        port = host[name]
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("puerto inválido: " + name)
        ports.add(port)
except (OSError, ValueError, TypeError, KeyError) as error:
    stop("No se pudo leer la configuración del MID-360: " + str(error))

def check_socket(ip, port, kind):
    protocol = "TCP" if kind == socket.SOCK_STREAM else "UDP"
    try:
        with socket.socket(socket.AF_INET, kind) as probe:
            if kind == socket.SOCK_STREAM:
                # Match the UI server: recent TCP connections may be in TIME_WAIT.
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind((ip, port))
    except OSError as error:
        if error.errno == errno.EADDRNOTAVAIL:
            stop(f"La IP Ethernet {ip} no está activa.\n"
                 "Comprueba la alimentación del MID-360, el cable Ethernet y el adaptador USB.\n"
                 f"En Ajustes del Sistema > Red, verifica la IPv4 manual {ip}; "
                 "para la configuración predeterminada, la máscara es 255.255.255.0.\n"
                 "Reconecta el adaptador y vuelve a ejecutar este script.")
        if error.errno == errno.EADDRINUSE:
            stop(f"El puerto {protocol} {ip}:{port} está ocupado.\n"
                 "Si hay otra sesión de ATLAS o Livox abierta, detenla primero "
                 "(Ctrl+C en su Terminal) y vuelve a intentarlo.")
        stop(f"No se pudo comprobar {protocol} {ip}:{port}: {error}")

check_socket(host_ip, 0, socket.SOCK_DGRAM)
for port in sorted(ports):
    check_socket(host_ip, port, socket.SOCK_DGRAM)
check_socket("0.0.0.0", 47777, socket.SOCK_STREAM)
print(f"Red local disponible: {host_ip}. Puertos principales de ATLAS libres.")

# Wi-Fi, phone tethering and USB can use different interfaces and addresses.
print("iPhone/iPad: usa Bonjour o la IP de la Mac en la red que comparten:")
for item in local_addresses():
    purpose = " (enlace del MID-360)" if item["address"] == host_ip else ""
    print(f"  {item['interface']}: {item['address']}{purpose}")
print("En ATLASViewer: Mapa > Ajustes > IP de la Mac. En hotspot usa la IP de la Mac, no la del teléfono.")

PY

if "$ATLAS_CHECK_ONLY"; then
  printf 'Comprobaciones locales correctas. No se inició ninguna captura.\n'
  exit 0
fi

# The supervisor forwards Ctrl+C to the probe's own process group, waits for its
# export, then starts geometry reconstruction. It never captures during --check.
if "$ATLAS_KEEP_SOURCE"; then
  exec python3 "$ATLAS_ROOT/tools/structure/run_session.py" --root "$ATLAS_ROOT" --keep-source
fi
exec python3 "$ATLAS_ROOT/tools/structure/run_session.py" --root "$ATLAS_ROOT"
