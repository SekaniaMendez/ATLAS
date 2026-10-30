#!/bin/bash
# Native macOS tests; listener binds an ephemeral loopback port only. No device,
# LiDAR, Bonjour advertisement, existing server, or standard preferences used.
set -euo pipefail
ATLAS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." >/dev/null && pwd -P)"
ATLAS_TEST_BUILD="$ATLAS_ROOT/build/iphone-connection-tests"
ATLAS_NETWORK="$ATLAS_ROOT/apps/ios/ATLASViewer/ATLASViewer/Networking"
mkdir -p -- "$ATLAS_TEST_BUILD"
xcrun swiftc -swift-version 5 \
  -module-cache-path "$ATLAS_TEST_BUILD/module-cache" \
  "$ATLAS_NETWORK/ConnectionEndpoint.swift" \
  "$ATLAS_ROOT/tests/ios/ConnectionEndpointTests.swift" \
  -o "$ATLAS_TEST_BUILD/endpoint-tests"
"$ATLAS_TEST_BUILD/endpoint-tests"
xcrun swiftc -swift-version 5 \
  -module-cache-path "$ATLAS_TEST_BUILD/module-cache" \
  "$ATLAS_NETWORK/ConnectionEndpoint.swift" \
  "$ATLAS_NETWORK/AtlasWireProtocol.swift" \
  "$ATLAS_NETWORK/AtlasConnection.swift" \
  "$ATLAS_ROOT/apps/ios/ATLASViewer/ATLASViewer/Models/LiveGeometryModel.swift" \
  "$ATLAS_ROOT/tests/ios/AtlasConnectionIntegrationTests.swift" \
  -o "$ATLAS_TEST_BUILD/connection-tests"
exec "$ATLAS_TEST_BUILD/connection-tests"
