# ATLAS Viewer for iPhone

Native SwiftUI + Metal viewer for point clouds processed by ATLAS on the Mac.
The app discovers the Mac with Bonjour (`_atlas._tcp`) and also supports a
manual IPv4 address. The binary TCP stream uses port `47777`.

On a physical iPhone, the viewer starts an ARKit world-tracking session, draws
the rear-camera image as a Metal YCbCr background, projects MID360 points using
the selected rigid calibration profile, and samples provisional RGB directly
from the camera image. Simulator builds retain the orbital LiDAR viewer because
ARKit camera capture requires physical hardware.

## Test without a MID-360

From the repository root:

```sh
cmake --preset mac-debug -DATLAS_ENABLE_UI_STREAM=ON
cmake --build --preset mac-debug
./build/mac-debug/apps/atlas_ui_demo
```

Then open `ATLASViewer.xcodeproj` in Xcode, select your development team and
your connected iPhone, and press Run. Accept the local-network permission. The
app should discover the demo automatically and render an animated sphere.

If Bonjour discovery does not find the Mac, enter the Mac's Wi-Fi IPv4 address
in the app and tap **Conectar**. Both devices must be on the same local network,
and macOS must allow incoming connections for the executable.

## Use the MID-360

```sh
cmake --preset mac-debug \
  -DATLAS_ENABLE_LIVOX=ON \
  -DATLAS_ENABLE_UI_STREAM=ON
cmake --build --preset mac-debug
./build/mac-debug/apps/atlas_livox_probe config/livox/mid360.json
```

The MID-360 may remain connected to the Mac over Ethernet while the iPhone and
Mac communicate over Wi-Fi. The current probe publishes decoded LiDAR frames;
when the SLAM stage is connected, its transformed map frames can use the same
stream protocol.

## Controls

- Drag to orbit the cloud.
- Pinch to zoom.
- **Demo** returns to the local sample cloud.
- Reflectivity controls the point color; Livox quality tags can mark suspect
  points in orange or red.
- The device menu selects a preliminary MID360-to-camera calibration profile.
  A physical iPhone automatically selects its known hardware identifier and the
  choice is persisted. Profiles live in
  `ATLASViewer/Resources/iphone-mid360-calibrations.json`.

## Camera calibration profiles

The included iPhone profiles are initial mounting estimates, not final optical
calibrations. `iphone14pro-mid360-v0` uses the supplied bracket measurements:
the MID360 body spans approximately 65–115 mm from the top of the phone. Every
profile remains marked `verified: false` until an RGB/LiDAR overlay is adjusted
against real edges and planes. ARKit supplies camera intrinsics at runtime;
these files describe only the rigid LiDAR-to-camera extrinsic transform.

The transform maps Livox coordinates into ARKit's native camera coordinates.
The current mount convention places the MID360 optical hemisphere (`+Z`) toward
the top of the iPhone, its connector toward the screen, and Livox `+X` toward
the rear-camera viewing direction. The GPU frustum test keeps only points in
front of the camera and inside the current ARKit image bounds.
The current RGB is a live overlay, so motion can expose timing error between
the Ethernet LiDAR frame and the latest camera frame. Persistent RGB maps will
require synchronized camera keyframes and SLAM poses on the Mac.

## Stream format (version 1)

Each TCP message starts with a 32-byte little-endian header. Point-cloud
messages are followed by 16 bytes per point: `Float32 x/y/z`, reflectivity,
Livox tag, and two reserved bytes. The Mac bounds the pending queue, filters
invalid or distant points, and limits each UI frame to keep latency stable.
