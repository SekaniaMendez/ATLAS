#!/bin/bash
# Native protocol/model tests; optional first argument is a C++ ATLS fixture.
set -euo pipefail
ATLAS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." >/dev/null && pwd -P)"
ATLAS_TEST_BUILD="$ATLAS_ROOT/build/iphone-geometry-tests"
mkdir -p -- "$ATLAS_TEST_BUILD"
xcrun swiftc -swift-version 5 \
  -module-cache-path "$ATLAS_TEST_BUILD/module-cache" \
  "$ATLAS_ROOT/apps/ios/ATLASViewer/ATLASViewer/Networking/AtlasWireProtocol.swift" \
  "$ATLAS_ROOT/apps/ios/ATLASViewer/ATLASViewer/Models/LiveGeometryModel.swift" \
  "$ATLAS_ROOT/tests/ios/LiveGeometryProtocolTests.swift" \
  -o "$ATLAS_TEST_BUILD/geometry-tests"
exec "$ATLAS_TEST_BUILD/geometry-tests" "$@"
