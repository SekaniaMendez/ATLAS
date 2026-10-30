#!/bin/bash
# Install geometry dependencies only inside this checkout's ignored build/ tree.
set -euo pipefail

ATLAS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." >/dev/null && pwd -P)"
ATLAS_ENV="$ATLAS_ROOT/build/structure-venv"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf '%s\n' 'Uso: ./scripts/setupGeometry.sh' \
    'Crea build/structure-venv con Open3D y las dependencias de geometría.' \
    'Necesita Python 3.12. Puedes elegirlo con ATLAS_GEOMETRY_PYTHON=/ruta/python3.12.'
  exit 0
fi
if [[ $# -ne 0 ]]; then
  printf 'Opción desconocida. Usa --help.\n' >&2
  exit 2
fi

compatible_python() {
  "$1" -c 'import sys; sys.exit(0 if sys.version_info[:2] == (3,12) else 1)' >/dev/null 2>&1
}

if [[ -x "$ATLAS_ENV/bin/python" ]]; then
  if ! compatible_python "$ATLAS_ENV/bin/python"; then
    printf 'El entorno existente usa una versión de Python no admitida: %s\n' "$ATLAS_ENV" >&2
    printf 'Conserva o renombra ese entorno y vuelve a ejecutar el instalador con Python 3.12.\n' >&2
    exit 1
  fi
else
  ATLAS_PYTHON=""
  if [[ -n "${ATLAS_GEOMETRY_PYTHON:-}" ]]; then
    ATLAS_PYTHON="$(command -v -- "$ATLAS_GEOMETRY_PYTHON" || true)"
    if [[ -z "$ATLAS_PYTHON" ]] || ! compatible_python "$ATLAS_PYTHON"; then
      printf 'ATLAS_GEOMETRY_PYTHON debe indicar un Python 3.12 ejecutable.\n' >&2
      exit 1
    fi
  else
    for ATLAS_CANDIDATE in python3.12 python3; do
      ATLAS_CANDIDATE_PATH="$(command -v -- "$ATLAS_CANDIDATE" || true)"
      if [[ -n "$ATLAS_CANDIDATE_PATH" ]] && compatible_python "$ATLAS_CANDIDATE_PATH"; then
        ATLAS_PYTHON="$ATLAS_CANDIDATE_PATH"
        break
      fi
    done
  fi
  if [[ -z "$ATLAS_PYTHON" ]]; then
    printf '%s\n' 'No se encontró Python 3.12; este entorno fija Python 3.12 para Open3D y SciPy.' \
      'Instala Python 3.12 o indica su ruta:' \
      '  ATLAS_GEOMETRY_PYTHON=/ruta/python3.12 ./scripts/setupGeometry.sh' >&2
    exit 1
  fi
  mkdir -p -- "$ATLAS_ROOT/build"
  "$ATLAS_PYTHON" -m venv "$ATLAS_ENV"
fi

"$ATLAS_ENV/bin/python" -m pip install --disable-pip-version-check -r "$ATLAS_ROOT/tools/structure/requirements.txt"
"$ATLAS_ENV/bin/python" -c 'import open3d, numpy, scipy, shapely; print("Geometría preparada: Open3D", open3d.__version__)'
