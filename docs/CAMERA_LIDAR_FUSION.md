# Cámara como evidencia complementaria del lienzo

El iPhone envía RGB al proceso `atlas_livox_probe --slam` por la misma conexión TCP
47777 que recibe la nube. La opción **Enviar color al filtro** controla el envío.
No hace falta abrir otro puerto. ARKit conserva la orientación nativa del sensor;
la imagen se reduce a 320 píxeles de ancho, a un máximo de 5 imágenes por segundo,
con una imagen en vuelo. Se descartan imágenes ante congestión, sin acumular vídeo.

Cada imagen incluye tiempo monotónico del teléfono, intrínsecos escalados, RGB
sRGB, transformación rígida LiDAR→cámara, estado de tracking y verificación del
perfil/lente. La cámara usa +X derecha, +Y arriba y -Z hacia delante. El mundo de
ARKit **no** se interpreta como el mundo del SLAM. La convención de imagen sigue
[imageResolution de ARKit](https://developer.apple.com/documentation/arkit/arcamera/imageresolution).

## Activación con el montaje real

Los perfiles existentes son semillas `verified: false`, aunque el montaje sea
rígido. El software recibe sus imágenes pero no las usa para evidencia hasta
verificar la transformación y la lente (`camera_lens: "wide"` para 1×).
La lente real seleccionada por ARKit debe coincidir; un cambio a 0.5× no hereda
la calibración 1×. Editar el perfil requiere reconstruir la app.

También hace falta medir la relación de relojes. El tiempo ARKit no es tiempo del
MID-360 ni tiempo Unix. La hora de llegada por Wi-Fi no establece sincronización.
Para una alineación medida, iniciar el proceso con estas tres opciones juntas:

```text
atlas_livox_probe config.json --slam \
  --camera-offset-ns OFFSET_MEDIDO \
  --camera-uncertainty-ns INCERTIDUMBRE_MAXIMA \
  --camera-lidar-clock unsynchronized
```

`tiempo_lidar = tiempo_telefono + OFFSET_MEDIDO`. El offset es entero con signo.
El reloj también puede ser `ptp` o `gps` si coincide con el que entrega el MID-360.
No usar cero como sustituto de una medición. La incertidumbre debe incluir el
error y deriva durante el recorrido; esta versión no estima offset ni deriva.
Reiniciar la sesión tras reiniciar un dispositivo o cambiar el montaje.

La selección de imagen exige `|tiempo_imagen - fin_scan| + incertidumbre ≤ 15 ms`;
cada retorno también debe estar a esa distancia temporal de la imagen. Se pueden
retener hasta 80 ms de datos para dar margen a la transmisión, sin bloquear el
hilo del sensor. Si llega tarde o no coincide, el retorno conserva solo geometría.
Se limitan las imágenes a 640×480 y el historial a seis exposiciones. Esto es una
primera fusión conservadora: puede colorear una fracción pequeña de los retornos.

## Efecto en el filtro

1. Se proyecta el retorno compensado al final del scan usando el perfil rígido.
   Solo se toma color delante de la cámara y dentro de la imagen.
2. Un buffer de profundidad por celdas de 4×4 píxeles rechaza retornos tras otro
   retorno más cercano, con tolerancia de 5 cm. No sustituye una máscara densa de
   oclusión: una oclusión que el LiDAR no haya muestreado puede no detectarse.
3. En cada celda candidata se compara cromaticidad RGB normalizada. Una diferencia
   superior a 0.25 reinicia su confirmación geométrica y exige tres imágenes
   distintas coherentes antes de promoverla. Reutilizar una imagen no suma
   evidencia. Colores oscuros, casi neutros o saturados no producen esta señal.
4. Si falta color, continúa la evidencia geométrica. Un bloqueo de color expira
   con la ventana temporal de 2 segundos; perder la cámara no paraliza el mapa.
5. Una celda estable nunca se borra por color. Los rayos LiDAR y poses fiables
   siguen siendo necesarios para devolver una zona al lienzo vacío.

Se conserva el RGB asociado al punto medido que se confirma. Las celdas ya
estables no se repintan retrospectivamente. Uniformar la intensidad de un color
no dispara la comparación; cambios reales de iluminación, reflejos o balance de
blancos todavía pueden retrasar candidatos. No se identifica un objeto ni se
infiere movimiento a partir de un solo color. No hay segmentación neuronal aún.

## Observación y formatos

La consola SLAM expone `colored` y `color_conflicts`; al cerrar, el stream informa
imágenes recibidas/rechazadas. Recibir una imagen no significa usarla: los gates
se aplican en la proyección. El arranque avisa si falta alineación temporal.
El mapa del iPhone muestra RGB donde hay evidencia; los demás puntos conservan
el color por reflectividad. PLY incluye `red`, `green`, `blue` y `has_color`;
el archivo de vóxeles ya conserva RGB. `slam_session.txt` guarda offset,
incertidumbre, tipo de reloj y contadores de fusión.

Uplink v1: cabecera little-endian `ATCR` (u32), versión u32, tamaño u32. Payload:
width/height/flags (u32), timestamp (u64), 16 doubles (fx/fy/cx/cy, R por filas, t
metros), RGB contiguo por filas desde arriba. Flags: bit 0 calibración y lente
verificadas; bit 1 tracking normal. Tamaño fijo de payload: 148 bytes. El servidor
limita el tamaño antes de reservar memoria y acepta fragmentación TCP.
Downlink: bit 1 de PointCloud indica xyz+RGB+validez en los mismos 16 bytes por
punto; bit 0 sigue indicando coordenadas del mapa. Actualizar app y Mac juntos.

La grabación raw de Mac sigue guardando LiDAR/IMU. Las imágenes en vivo **no** se
incorporan automáticamente a esa grabación: el registro del teléfono es separado,
y el replay actual conserva el filtro geométrico sin reconstruir evidencia RGB.

Pruebas: proyección y orientación, oclusión, tiempo por retorno, offset negativo,
reloj incorrecto, calibración/rotación inválida, exposición repetida, cambio de
brillo, retención de candidatos, borrado solo geométrico, RGB a archivo/PLY,
paquetes fragmentados, límites, reconexión y parada del receptor. Falta validar
reproyección y sincronización sobre el montaje físico.

## Prueba física del enlace — 28 de octubre de 2026

Con el iPhone 14 Pro y el MID-360 conectados, una captura de 90 segundos recibió
289 imágenes RGB sin rechazos y envió al teléfono 199 snapshots del mapa
(7.510.146 puntos acumulados entre los envíos). El motor calculó 892 poses
aceptadas y el mapa exportado contiene 44.882 voxels. Los snapshots repiten
geometría; ese total transmitido no representa puntos únicos del mapa.

Esta prueba confirma el transporte bidireccional durante el mapeo, pero
`colored=0` y `color_conflicts=0`: faltaban la calibración extrínseca y la
alineación temporal verificadas. No valida aún la proyección de color ni la
detección de movimiento por RGB. La evidencia local está en
`build/hardware-validation/corrected-live-20261028-160629/`.
