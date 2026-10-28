# Capturas, almacenamiento y sincronización — fase 2

Esta entrega permite grabar datos crudos, reproducirlos sin hardware, guardar un
mapa y descargar bloques a disco con una caché acotada. Incluye sincronizadores
independientes de LiDAR, IMU y cámara y un orquestador temporal. Todavía no estima
trayectoria ni proyecta automáticamente puntos crudos al marco del mapa.

## Grabar con el MID-360

Desde la raíz, con el sensor y la configuración de red habitual:

```sh
cmake --preset mac-debug -DATLAS_ENABLE_LIVOX=ON -DATLAS_ENABLE_UI_STREAM=ON -DATLAS_BUILD_TESTS=ON
cmake --build --preset mac-debug
mkdir -p captures
./build/mac-debug/apps/atlas_livox_probe config/livox/mid360.json --record captures/levantamiento-01
```

El directorio de la sesión **no debe existir**; nunca se sobrescribe una sesión.
Ctrl+C detiene la fuente y drena la grabación. Se guarda el bus crudo, antes del
filtro de proximidad y de cualquier reducción de puntos del visor. La grabación
incluye coordenadas/reflectividad/tags, offsets por punto, duración de paquete,
reloj y timestamp originales, datos IMU y eventos de estado. Se conserva una
copia textual de la configuración Livox y la hora de creación de la sesión.

`--metadata archivo` adjunta el contenido de una calibración/configuración externa.
Si no se proporciona, la sesión declara que no dispone de esa calibración; no
inventa una transformación. El estado de calibración del iPhone se guarda junto
a las imágenes en el propio teléfono.

`CaptureRecorder` escribe desde un hilo dedicado. Su cola reserva como máximo
32 MiB de payload codificado, incluyendo el registro en escritura; el bookkeeping,
el dato fuente y una codificación temporal son adicionales. Si la cola se llena o
falla el disco, la captura se marca fallida y el probe se detiene con error. No se
descartan silenciosamente mediciones para simular una grabación completa.

## Reproducir sin sensor

```sh
./build/mac-debug/apps/atlas_capture inspect captures/levantamiento-01
./build/mac-debug/apps/atlas_capture replay captures/levantamiento-01 --speed 1 --ui
```

El lector trabaja registro a registro. Reproduce el orden de recepción en el Mac;
`--speed` escala los intervalos de llegada del reloj monotónico del Mac. No ordena
LiDAR y cámara según relojes crudos que podrían pertenecer a dispositivos distintos.
`--speed 0` ejecuta sin pausas, útil para pruebas o procesamiento offline. El visor
es una salida reducida; no se usa para validar conservación de datos de captura.
La reproducción actual reejecuta el preprocesador para mostrar la nube LiDAR. Los
metadatos de cámara están disponibles a través de `CaptureReader`/`CaptureEvent`;
este CLI no reconstruye ni reproduce el color de las imágenes externas.

Prueba sintética, sin sensor:

```sh
./build/mac-debug/apps/atlas_capture synth captures/sintetica-01
./build/mac-debug/apps/atlas_capture inspect captures/sintetica-01
./build/mac-debug/apps/atlas_capture replay captures/sintetica-01 --speed 0
./build/mac-debug/apps/atlas_capture map-demo captures/mapa-demo-01
```

La captura sintética contiene 200 eventos y 10.000 puntos. `map-demo` usa una
caché de solo dos bloques para forzar expulsiones y recargas. Son herramientas
de verificación, no una trayectoria simulada ni un sistema SLAM.

## Cámara en iPhone

El botón **Grabar cámara** guarda keyframes JPEG en sus coordenadas nativas,
intrínsecos, pose local de ARKit, dimensiones, estado de tracking y timestamps.
La sesión copia el perfil extrínseco seleccionado y su campo `verified`. Se
deshabilita cambiar el perfil o lente durante la grabación. Pausar o reiniciar
ARKit cierra la sesión de captura para no mezclar marcos de tracking.

Hay como máximo una imagen en codificación y cinco keyframes por segundo. Las
omisiones por escritor ocupado se contabilizan. Es una captura de keyframes,
no video completo. La finalización escribe `complete.json`; si hay un error no
se declara completa. No cerrar forzosamente la app antes de ver el estado final.

Los archivos quedan en **Archivos → En mi iPhone → ATLAS Viewer → ATLASCaptures**,
también disponibles mediante la compartición de archivos de Finder. Copiar la
carpeta completa junto a la captura del Mac conserva imágenes y calibración.
No se implementó transporte automático de imágenes al Mac ni importación/fusión
automática de las dos carpetas. `CameraFrame` es el contrato C++ para que un futuro
importador entregue esos metadatos al bus, grabador u orquestador.

`timestamp_s` es el tiempo monotónico de ARKit. `encoded_unix_s` registra cuándo
se escribió el metadato, no cuándo se expuso la imagen ni una correspondencia de
relojes. Los relojes iPhone/Mac todavía necesitan una alineación medida.

## Persistencia del mapa

`VoxelMapArchive::save(new_directory, map)` guarda un snapshot completo.
`VoxelMapArchive::load(directory, max_payload_bytes)` crea un mapa en RAM conservando
resolución, límites, claves, conteos, revisión, color y orden del buffer circular.
El consumidor recibe un reinicio y todos los bloques como cambiados al cargar.

El formato es explícitamente little-endian, con magic/version, longitudes máximas,
CRC32 por registro y marcador final. No serializa el layout C++ ni punteros. Los
lectores rechazan versiones desconocidas, corrupción, truncación, claves inválidas
y conteos inconsistentes. El límite de carga de snapshots contabiliza payload de
voxels/muestras, no RSS, bookkeeping ni el registro temporal de lectura (hasta
64 MiB). Un snapshot es una carga completa; para mapas grandes se usa la caché.

```cpp
#include <atlas/storage/PagedVoxelMap.hpp>

using atlas::storage::PagedVoxelMap;
PagedVoxelMap map("captures/mapa-01", PagedVoxelMap::Mode::Create,
                  {.voxel_size_m = 0.05, .max_samples_per_voxel = 32},
                  64 * 1024 * 1024);
map.insert({.x_m = 1.0, .y_m = 2.0, .z_m = 0.5}); // ya en marco del proyecto
map.flush(); // comprobar errores antes de dar la sesión por guardada
```

`PagedVoxelMap` mantiene un hash de bloques residentes y una lista LRU; expulsa el
menos recientemente usado cuando debe cargar otro. Guarda primero los bloques
modificados. Una escritura fallida conserva el bloque sucio en RAM y lanza un
error. `get_voxel` devuelve una copia para evitar punteros invalidados por expulsión.

El presupuesto reserva conservadoramente el payload máximo por bloque, incluso
si tiene pocas observaciones. No es un límite del proceso completo: la caché de
SQLite (configurada a aproximadamente 1 MiB), sus estructuras, las copias solicitadas
por consumidores y buffers temporales de un bloque son adicionales. La cantidad
de bloques residentes sí se mantiene dentro del límite calculado. No existe una
lista en RAM de todos los bloques del archivo.

SQLite se usa únicamente para persistencia y transacciones. Se requiere SQLite3
de desarrollo al compilar; se usa el del SDK de macOS aquí. Las consultas calientes
siguen siendo por hash; una consulta fría hace I/O y no tiene costo O(1) total.
`flush()` confirma todos los bloques sucios residentes en una transacción; cada
expulsión también guarda su bloque de forma transaccional. Tras un cierre abrupto
solo se garantizan las escrituras ya confirmadas. El destructor intenta guardar,
pero el llamador debe usar `flush()` para enterarse de un error.

Para reabrir, usar `Mode::Open`; resolución y límites se recuperan del archivo.
Se admite un escritor por directorio. `.writer-lock` impide aperturas simultáneas
por esta API. Un proceso terminado forzosamente puede dejar ese directorio vacío:
antes de retirarlo manualmente, verificar que ningún proceso tenga abierto el mapa.

## Sincronizadores independientes y orquestador

`LidarSynchronizer`, `ImuSynchronizer` y `CameraSynchronizer` son instancias tipadas
de `StreamSynchronizer<T>`. Cada una conserva su cola y alineación de reloj. El
orquestador es un `ISensorDataSink`, por lo que se puede suscribir al bus crudo o
alimentar con eventos reproducidos. Esta entrega no lo conecta a un estimador.

La transformación temporal se configura mediante `ClockAlignment`:

```text
t_referencia = ancla_referencia + tasa × (t_sensor − ancla_sensor)
```

Cada alineación identifica el sensor y tipo de reloj de origen y declara su
incertidumbre. Todas deben referirse al mismo reloj/sesión de destino. Para flujos
del mismo reloj de dispositivo verificado se puede usar tasa 1 y anclas iguales;
que dos dispositivos declaren `Unsynchronized` o `Unknown` no significa que sus
relojes coincidan. El orquestador no estima automáticamente offsets por llegada de
red. Se restan anclas enteras antes de aplicar la tasa para conservar diferencias
pequeñas en timestamps grandes; el resultado se redondea a nanosegundos.

`configure_clocks(lidar_alignment, imu_alignment, camera_alignment)` valida todos
los modelos antes de cambiarlos. Cambiarlos vacía las tres colas e incrementa una
época; ningún paquete mezcla datos de modelos antiguos y nuevos. Los tiempos
originales permanecen dentro de las muestras; los normalizados se almacenan aparte.

`push()` devuelve explícitamente aceptación o rechazo por reloj no alineado,
sensor/reloj incorrecto, tiempo inválido, orden no creciente o cola llena. Los
duplicados y datos tardíos se rechazan, no se reordenan. Un reinicio de reloj
requiere reconfiguración. El orquestador limita colas, puntos por paquete y bytes
de metadatos de cámara; `stats()` expone rechazos y datos descartados al reiniciar.

`poll(now_ns)` toma el intervalo temporal del primer paquete LiDAR y busca muestras
IMU que lo encierren, comprobando que no haya huecos mayores al configurado. Busca
el keyframe más cercano al centro del intervalo dentro de una tolerancia. La
cámara puede ser opcional o requerida. `now_ns` pertenece al reloj de referencia;
no pasar sin conversión el reloj de pared del Mac.

Si faltan datos, espera hasta el plazo configurado. `poll(now_ns, true)` drena al
final del archivo. Los paquetes incompletos se entregan con indicadores explícitos;
`ready_for_estimation` exige cobertura IMU, incertidumbre temporal aceptable y
cámara si fue requerida. Alinear tiempos no interpola IMU, no compensa el movimiento
por punto y no estima extrínsecos ni poses: esas tareas siguen en las fases siguientes.

## Evidencia y límites de verificación

- Ocho pruebas pasan en Debug y Release, incluyendo las del lienzo anterior.
- Capturas: roundtrip, productores concurrentes, metadata de cámara, overflow,
  archivos truncados, CRC y protección contra sobrescritura.
- Mapas: snapshot, continuación del buffer circular, recarga de 24 bloques con
  solo dos residentes, reapertura y recuperación después de un commit fallido.
- Sincronización: relojes independientes, colas, datos tardíos, nanosegundos en
  timestamps grandes, cobertura IMU, cámara requerida, timeout y reconfiguración.
- Las pruebas nuevas pasan con AddressSanitizer y UndefinedBehaviorSanitizer.
- Compilan el probe con SDK Livox y la app iOS para Simulator. La captura real de
  cámara y LiDAR y la precisión temporal requieren una prueba con los dispositivos.

Las grabaciones incompletas conservan el archivo pero el lector actual las rechaza;
todavía no existe una herramienta de salvamento del prefijo válido. El timestamp
de recepción permite reproducir la llegada, no demuestra sincronización física.
