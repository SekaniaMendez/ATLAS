#!/bin/bash
# Read and record Z1 positions over its local Wi-Fi; never configure the receiver.
set -euo pipefail
ATLAS_GNSS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null && pwd -P)"
exec python3 "$ATLAS_GNSS_ROOT/tools/gnss/record_z1.py" "$@"
