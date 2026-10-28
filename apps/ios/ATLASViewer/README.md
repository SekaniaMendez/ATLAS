# ATLAS Viewer for iPhone

Native SwiftUI + Metal viewer for point clouds processed by ATLAS on the Mac.
The app discovers the Mac with Bonjour (`_atlas._tcp`) and also supports a
manual IPv4 address. The binary TCP stream uses port `47777`.

For sensor-frame LiDAR on a physical iPhone, the viewer starts ARKit and offers
provisional camera RGB or a calibration overlay using the selected rigid
LiDAR-to-camera profile. SLAM maps use an orbital view on both iPhone and
Simulator. Their colors represent reflectivity, not camera RGB; the phone does
not need to be mounted or aligned with the LiDAR to inspect the map.

## Test without a MID-360

From the repository root:

```sh
cmake --preset mac-debug -DATLAS_ENABLE_UI_STREAM=ON
cmake --build --preset mac-debug
./build/mac-debug/apps/atlas_ui_demo
```

Then open `ATLASViewer.xcodeproj` in Xcode, select your development team and
your connected iPhone, and press Run. Accept the local-network permission. The
app should discover the demo automatically. Tap **Ocultar cámara** to inspect
the animated sphere in the orbital view on a physical iPhone.

If Bonjour discovery does not find the Mac, enter the Mac's Wi-Fi IPv4 address
in the app and tap **Conectar**. Both devices must be on the same local network,
and macOS must allow incoming connections for the executable.
USB lets Xcode install and launch the app; it does not create a tunnel for this
TCP stream. The viewer still needs the Mac's reachable network address.

## Use the MID-360

```sh
cmake --preset mac-debug \
  -DATLAS_ENABLE_LIVOX=ON \
  -DATLAS_ENABLE_UI_STREAM=ON
cmake --build --preset mac-debug
./build/mac-debug/apps/atlas_livox_probe config/livox/mid360.json
```

The MID-360 may remain connected to the Mac over Ethernet while the iPhone and
Mac communicate over Wi-Fi. This command publishes filtered points in the
sensor frame and retains the camera modes described below.

## View the local SLAM map

Use an optimized build for live SLAM:

```sh
cmake --preset mac-release -DATLAS_ENABLE_LIVOX=ON -DATLAS_ENABLE_UI_STREAM=ON
cmake --build --preset mac-release
./build/mac-release/apps/atlas_livox_probe config/livox/mid360.json --slam
```

Keep the sensor still during initialization, then move it slowly while retaining
overlap with observed surfaces. Rebuild the iPhone app from this project so it
recognizes world-frame snapshots. The heading changes to **ATLAS · Mapa SLAM**;
the point count and advancing `frame` counter confirm reception. Check the
visible geometry to verify rendering: **Conectado a ATLAS** alone confirms TCP.
The separate SLAM indicator reports **Inicializando**, **Seguimiento activo**
or **Seguimiento perdido**. A lost state leaves the last valid map visible;
the frame counter may still advance as that snapshot is retransmitted. Expand
**Detalle del seguimiento** for the estimator's reason and accepted/rejected
scan counts. A temporary rejection can recover; consult the specific reason
before restarting. Older servers without telemetry show **Estado no disponible**.

Each snapshot replaces the displayed map, with up to 50,000 sampled points
every 250 ms. The full saved map is unaffected by this display limit. Camera
overlay controls are disabled for world coordinates; camera recording remains
independent. This is a local metric map, not a georeferenced or RGB map. See the
[SLAM prototype guide](../../../docs/SLAM_PROTOTYPE.md) for recording, replay,
exports and current tracking limitations.

## Controls

- Drag to orbit and pinch to zoom in SLAM mode or when the camera view is off.
- **Demo** returns to the local sample cloud.
- In the orbital view, reflectivity controls point color. Sensor-frame Livox
  quality tags can mark suspect points in orange or red. Meaningful camera RGB
  alignment needs a suitable fixed mounting and calibration verification.
- The device menu selects a preliminary MID360-to-camera calibration profile.
  A physical iPhone automatically selects its known hardware identifier and the
  choice is persisted. Profiles live in
  `ATLASViewer/Resources/iphone-mid360-calibrations.json`.

## Camera calibration profiles

These profiles apply to sensor-frame camera modes, not the orbital SLAM map.
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

## Record camera keyframes

Tap **Grabar cámara**, then **Detener captura** and wait for completion. Native
JPEG keyframes (up to 5 fps, one image in flight), ARKit timestamps/intrinsics/
local poses, tracking status and the selected calibration profile are saved under
Files → On My iPhone → ATLAS Viewer → ATLASCaptures. Lens/profile changes are
disabled during capture. Pausing/restarting ARKit ends the capture session.

Copy the whole folder alongside the Mac raw recording. These are separate clock
domains: images are not yet sent to the Mac or automatically synchronized/fused
with LiDAR. A complete camera capture has `complete.json`. See
[capture and synchronization](../../../docs/CAPTURE_AND_SYNCHRONIZATION.md).

## Stream format (version 1)

Each TCP message starts with a 32-byte little-endian header. Point-cloud
messages are followed by 16 bytes per point: `Float32 x/y/z`, reflectivity,
Livox tag, and two reserved bytes. For point-cloud messages, header flag bit 0
marks a complete snapshot in the fixed SLAM frame. The receiver replaces the
previous map, including when an empty snapshot clears it. The Mac also replaces
pending world snapshots instead of concatenating them. Sensor-frame streams
filter invalid or distant points; world snapshots filter invalid coordinates
without applying a distance cutoff around the initial origin. Both modes bound
the pending queue and the number of displayed points.

SLAM telemetry uses the previously reserved `SensorStatus` type `2`, with
header flags `1` identifying the SLAM subtype. Its versioned payload carries
tracking state, fatal-error flag, accepted/rejected scan counts and bounded
UTF-8 diagnostic text. Earlier viewers recognize and ignore type `2`, so
telemetry does not interrupt their point stream. It is independent of sensor
warning banners and is sent alongside map snapshots, including the final state.

Sensor-warning messages use type `3` and carry a 12-byte payload: warning code,
severity, measured value, and threshold. The viewer currently presents these
as a temporary banner. Abrupt IMU acceleration/rotation warnings are available
immediately. The protocol supports the `2.5 m/s` warning, but the current SLAM
engine does not yet feed its velocity estimate into that warning path.
