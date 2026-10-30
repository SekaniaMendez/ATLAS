# ATLAS Viewer for iPhone

Native SwiftUI + Metal viewer for point clouds processed by ATLAS on the Mac.
The app discovers the Mac with Bonjour (`_atlas._tcp`) and also supports a
manual IPv4 address. The binary TCP stream uses port `47777`.

For sensor-frame LiDAR on a physical iPhone, enabling the camera in Settings
starts ARKit and offers provisional camera RGB or a calibration overlay using the selected rigid
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
app should discover the demo automatically. The camera starts disabled, so the
animated sphere can be inspected in the orbital view on a physical iPhone.

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

### Live surfaces and reference axes

The updated Mac server also extracts measured plane meshes on a separate worker
about once per second. ATLASViewer draws them in the same orbital map frame,
with a distinct color per surface and local X/Y/Z axes in red/green/blue. Use
**Puntos / Superficies / Ejes** to choose the visible layers. The geometry indicator
shows surface and triangle counts and the age of the most recent replacement.

This is a bounded planar preview: curves and unsupported details remain visible
as points. The final offline reconstruction includes finer residual surfaces.
No camera overlay or geographic frame is inferred from these local axes. IDs and
axes are recomputed per snapshot and are not persistent structural identities.

An empty map, tracking loss or reconnect clears the geometry. A completed old
calculation cannot restore a cleared snapshot. After three seconds without a new
geometry update the meshes disappear; repeating a frozen map timestamp does not
refresh them. Queues retain only the latest pending map and geometry message.

To test from saved measurements without starting the Livox SDK:

```sh
./build/mac-release/apps/atlas_geometry_preview \
  --input output/20261028-172733/mapa.ply
```

Connect the app to the Mac as usual. The preview simulates advancing capture
timestamps; `--exercise-clears` adds empty/lost/recovery transitions. It leaves
the original file untouched. See [structural geometry](../../../docs/STRUCTURAL_GEOMETRY.md)
for limits, installation and reproducible tests.

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

Geometry uses `SensorStatus` type `2`, subtype/header flags `2`; older viewers
ignore this subtype without disrupting their point stream. The header count is
the surface count, and timestamp identifies the source SLAM map. Payload v1 is:

| Offset | Type | Meaning |
| ---: | --- | --- |
| 0 | uint16 | Geometry version, 1 |
| 2 | uint16 | Bit 0 complete replacement (required), bit 1 budget-limited |
| 4, 8, 12 | uint32 × 3 | Surface, vertex and index counts |
| 16 | uint64 | Source revision, increasing for replacements and clears |
| 24 | uint32 | Input point count |
| 28 | float32 | Extraction time in milliseconds |

Next are 84-byte surface records: ID, kind (1 horizontal, 2 vertical, 3 inclined,
4 reserved freeform), origin XYZ, 9 basis floats in column order, area, fit RMS,
support count, first vertex, vertex count, first index, index count. Then come all
packed XYZ Float32 vertices **in map coordinates** and UInt32 indices **local to
each surface's vertex range**. All values are little-endian. Maximum counts are
32 surfaces / 30,000 vertices / 90,000 indices; the decoder validates payload
lengths, finite values, right-handed orthonormal frames and triangle indices
before replacing the model. Zero surfaces/vertices/indices is a valid clear.

## Flujo actual: Mapa y Etiquetar

La app tiene dos pestañas. **Mapa** muestra el SLAM y sus capas de puntos,
superficies y ejes; conexión, calibración y grabación están en **Ajustes**.
La cámara ya no arranca al abrir la app: se activa expresamente desde
**Mapa → Ajustes → Cámara**. Ver el mapa y etiquetar no requiere cámara.

**Etiquetar** abre el editor de capturas guardadas en la Mac. Desde la raíz del
proyecto ejecuta:

```sh
./startLabeling.sh --lan
```

Con ambos dispositivos en la misma red, pega el enlace completo de emparejamiento
que muestra el servidor en **Etiquetar → Conectar con la Mac**. Usa el puerto
`8770` de forma predeterminada, independiente del mapa TCP `47777`. Bonjour
`_atlas-labels._tcp` encuentra la dirección; la clave del primer emparejamiento
va en el enlace `#token=…` y la app la guarda en Keychain.

Si la Mac usa internet compartido del teléfono, el enlace debe llevar la IP de
la **Mac** en esa conexión, no la puerta de enlace del teléfono ni la dirección
Ethernet del LiDAR. El enlace manual permite conectar cuando Bonjour no aparece.
La configuración contempla este caso, pero la prueba física completa con internet
compartido sigue pendiente.

Selecciona grupos, revisa las propuestas de suelo/pared/techo y asigna categorías
de objetos. Varias selecciones etiquetadas juntas comparten una instancia. Las
etiquetas se guardan en la Mac; exportar produce NPZ + JSON con objetivos humanos
y mantiene `-1` en los puntos sin etiqueta. Esta etapa prepara datos y **no entrena
todavía una red neuronal**. Los originales de `output/` permanecen intactos.

En **Etiquetar**, las capas separan suelo, paredes, techo, objetos y puntos sin
agrupar; el techo está oculto al abrir una captura. El visor permanece visible y
las herramientas se desplazan en su propio panel. Apple Pencil selecciona puntos;
los dedos navegan en **Girar / elegir grupo**. **Pintar puntos** permite seleccionar
también con ratón o dedo. El borrador admite borrar, deshacer trazos y crear un
objeto etiquetado nuevo. Recargar desde la cabecera de la app pide confirmación
si quedan puntos seleccionados sin guardar.

**Nueva categoría**, junto al selector **Categoría**, permite añadir un nombre de
hasta 80 caracteres desde el iPad o iPhone. Al crearla se selecciona para el próximo
guardado; todavía debes pulsar **Guardar etiqueta** o **Crear objeto etiquetado**.
Un nombre repetido reutiliza la categoría existente. El catálogo se comparte entre
capturas y se actualiza al abrir el formulario, conservando el borrador y las notas.

Respalda juntos `output/categories.json` y `output/.categories.json.identity`,
además de las anotaciones, con el servidor detenido. Los IDs de categoría son
estables dentro de ese catálogo; cada exportación guarda su propia copia fija de
la taxonomía en el JSON. Añadir clases después no modifica exportaciones anteriores
ni inicia un entrenamiento.

El 29 de octubre de 2026, la nueva app universal se compiló e instaló en el
**iPhone 14 Pro** y el **iPad Air 5 con M1**, con Modo de desarrollador activado.
El iPad se abrió y emparejó con el servidor local; el usuario confirmó la captura
y los grupos visibles. La apertura remota del iPhone quedó pendiente de desbloquear
su pantalla. En un dispositivo nuevo puede ser necesario confiar en el perfil en
**Ajustes → General → VPN y gestión de dispositivos → perfil del desarrollador →
Confiar** y permitir el acceso a la red local.

Consulta [Etiquetado semántico](../../../docs/SEMANTIC_LABELING.md) para revisión,
exportación, límites de los grupos y conservación de futuras capturas con
`./startAtlas.sh --keep-source`.
