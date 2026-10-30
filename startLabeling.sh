#!/bin/bash
# Label existing captures. No sensor, camera or neural-network training is started.
set -euo pipefail
ATLAS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null && pwd -P)"
ATLAS_PYTHON="$ATLAS_ROOT/build/structure-venv/bin/python"
if [[ ! -x "$ATLAS_PYTHON" ]]; then
  printf 'Prepara las dependencias con: %s/scripts/setupGeometry.sh\n' "$ATLAS_ROOT" >&2
  exit 1
fi
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
exec "$ATLAS_PYTHON" "$ATLAS_ROOT/tools/structure/prepare_annotations.py" --root "$ATLAS_ROOT" "$@"
