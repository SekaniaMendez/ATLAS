#!/bin/bash
# Reconstruct an existing measurement snapshot without starting any hardware.
set -euo pipefail
ATLAS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null && pwd -P)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf '%s\n' 'Uso: ./createGeometry.sh archivo.ply [directorio_nuevo]' \
    'Crea superficies, marcos, GLB y visor 3D sin conectar el Livox.' \
    'Conserva siempre la captura original. El destino no debe existir.'
  exit 0
fi
if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf 'Uso: ./createGeometry.sh archivo.ply [directorio_nuevo]\n' >&2
  exit 2
fi
ATLAS_PYTHON="$ATLAS_ROOT/build/structure-venv/bin/python"
if [[ ! -x "$ATLAS_PYTHON" ]]; then
  printf 'Prepara las dependencias con: %s/scripts/setupGeometry.sh\n' "$ATLAS_ROOT" >&2
  exit 1
fi
ATLAS_DESTINATION="${2:-$(dirname -- "$1")/geometria-$(date +%Y%m%d-%H%M%S)}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
exec "$ATLAS_PYTHON" "$ATLAS_ROOT/tools/structure/analyze.py" \
  --input "$1" --output "$ATLAS_DESTINATION"
