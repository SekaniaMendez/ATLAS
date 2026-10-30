# Calibración medida de cámara y MID-360

`solve_extrinsics.py` calcula **una propuesta no verificada** de posición y
orientación de la cámara respecto al LiDAR usando correspondencias medidas.
No instala perfiles ni activa RGB. El montaje anterior deja de ser válido al
girar el MID-360, mover el teléfono, cambiar de lente o modificar el soporte.

## Entorno y ejecución

Desde la raíz del proyecto, con Python 3.12:

```sh
python3.12 -m venv build/calibration-venv
build/calibration-venv/bin/python -m pip install -r tools/calibration/requirements.txt
build/calibration-venv/bin/python tools/calibration/solve_extrinsics.py \
  output/calibracion/observaciones.json --output output/calibracion/propuesta.json
build/calibration-venv/bin/python -m unittest discover -s tests/calibration -p 'test_extrinsics.py' -v
```

El archivo de salida debe ser nuevo y su directorio debe existir. No se
sobrescriben resultados. Código de salida: `0` propuesta que supera los filtros
numéricos y aún necesita revisión; `1` entrada inválida/error de ajuste o E/S;
`2` propuesta guardada pero rechazada por calidad. **En todos los resultados
`verified` permanece `false`.** El JSON incluye SHA-256 del archivo de entrada.

## Capturar las observaciones

1. Fijar rígidamente ambos sensores en su montaje definitivo. Mantener la lente
   1× y el mismo modo de cámara durante toda la sesión. No cambiar la colocación
   del teléfono entre calibración y escaneo.
2. Usar un tablero rígido mate con esquinas exteriores visibles o un objeto con
   vértices geométricos inequívocos. Las esquinas impresas dentro de un damero
   no son esquinas geométricas visibles al LiDAR.
3. Registrar simultáneamente JPEG nativo y nube en **coordenadas del sensor**.
   Mantener el conjunto y la escena quietos durante cada toma de 5–10 segundos;
   acumular sólo puntos de ese intervalo. No usar el mapa SLAM, puntos ya
   transformados al mundo ni retornos de instantes en movimiento.
4. Recoger unas diez posiciones con distancias, inclinaciones y ubicaciones
   variadas dentro de la imagen. Separar posiciones completas en `train` y
   `validation` antes de ajustar. La validación no debe repetir una misma toma
   con nombres o esquinas distintos.
5. Señalar el mismo vértice en la imagen y en la nube, conservando un identificador
   por pareja. Para mejorar precisión, ajustar bordes/planos en regiones medidas
   y usar sus intersecciones, en vez de elegir un retorno aislado junto al borde.
   Revisar manualmente que los emparejamientos describan el mismo punto físico.

El solver consume los pares; no detecta automáticamente tableros, no infiere
correspondencias ni certifica que el usuario estuvo inmóvil. El selector de
etiquetas de objetos no es por sí solo un recolector de pares 3D–2D.

## Preparar y usar el selector de correspondencias

Exportar un intervalo estático por vista con
[`atlas_calibration_export`](../../apps/calibration_export/README.md). Cada carpeta
LiDAR debe contener `cloud.ply` ASCII y `metadata.json` que declare
`coordinate_frame: "lidar_sensor"`, `units: "m"`. Se rechazan mapas SLAM. El
preparador acepta hasta 500.000 puntos únicos por vista; recortar el intervalo de
exportación si es demasiado grande, manteniendo visibles los vértices necesarios.

Copiar la captura del iPhone a la Mac conservando JPEG, JSON por imagen y
`session.json` en la misma carpeta. Crear un manifiesto que empareje cada intervalo
LiDAR con una imagen de **la misma colocación inmóvil**:

```json
{
  "camera_lens": "wide",
  "views": [
    {"id": "pose-01", "split": "train", "lidar_directory": "lidar/pose-01", "camera_frame": "iphone/00000005.json"},
    {"id": "pose-02", "split": "train", "lidar_directory": "lidar/pose-02", "camera_frame": "iphone/00000040.json"},
    {"id": "pose-03", "split": "train", "lidar_directory": "lidar/pose-03", "camera_frame": "iphone/00000075.json"},
    {"id": "pose-04", "split": "validation", "lidar_directory": "lidar/pose-04", "camera_frame": "iphone/00000110.json"},
    {"id": "pose-05", "split": "validation", "lidar_directory": "lidar/pose-05", "camera_frame": "iphone/00000145.json"}
  ]
}
```

Las rutas relativas se resuelven desde la carpeta del manifiesto, no desde el
directorio de la terminal. `camera_frame` es el JSON nativo exportado por ATLAS;
el preparador toma K, resolución, nombre del JPEG y tiempo de ese archivo, además
de comprobar lente `wide` en su `session.json`. Los nombres anteriores son sólo
ejemplos: elegir las imágenes reales de cada intervalo después de revisarlas.

```sh
build/calibration-venv/bin/python tools/calibration/prepare_workspace.py \
  output/calibracion/manifest.json --output output/calibracion/selector
build/calibration-venv/bin/python -m http.server 8780 \
  --bind 127.0.0.1 --directory output/calibracion/selector
```

Abrir `http://127.0.0.1:8780/collector.html` en la Mac. La carpeta de salida debe
ser nueva. El servidor se limita a localhost; este comando no publica la captura
en la red del iPad. El selector muestra la imagen nativa y la nube con controles
de giro, zoom y desplazamiento. Elegir un píxel y el retorno 3D que representa el
mismo vértice, pulsar **Añadir par** y repetir. Revisar las tomas y marcar la
confirmación de posición inmóvil en cada vista. Al menos cuatro pares por vista;
distribuirlos ampliamente en la escena y en la imagen.

Los retornos seleccionados son aproximaciones al vértice físico, no
intersecciones de planos ajustadas automáticamente. Comprobar los bordes desde
varios ángulos en el visor. El borrador se conserva en ese navegador; **Descargar
correspondencias** genera el JSON que se pasa al solver. **Reanudar** permite
importar un borrador correspondiente a las mismas imágenes y nubes. Descargar o
confirmar una toma no marca la calibración como verificada.

## Esquema `observaciones.json`

Ejemplo de **una vista**; un documento ejecutable necesita al menos tres vistas
de ajuste y dos de validación, cada una con cuatro o más pares:

```json
{
  "schema_version": 1,
  "camera_lens": "wide",
  "image_coordinates": "native_unrotated",
  "views": [
    {
      "id": "pose-01",
      "split": "train",
      "static_pair_confirmed": true,
      "image": "00000000.jpg",
      "width": 1920,
      "height": 1440,
      "intrinsics_row_major": [1400, 0, 960, 0, 1400, 720, 0, 0, 1],
      "correspondences": [
        {"id": "superior-izquierda", "xyz_lidar_m": [0.1, 2.0, 0.8], "uv_px": [510, 300]},
        {"id": "superior-derecha", "xyz_lidar_m": [0.1, 2.8, 0.8], "uv_px": [930, 300]},
        {"id": "inferior-derecha", "xyz_lidar_m": [0.1, 2.8, 0.2], "uv_px": [930, 730]},
        {"id": "inferior-izquierda", "xyz_lidar_m": [0.1, 2.0, 0.2], "uv_px": [510, 730]}
      ]
    }
  ]
}
```

Estos números sólo ilustran el formato; **no son mediciones ni una calibración**.

- `camera_lens`: `wide` o `ultraWide`. No mezclar lentes en un documento. Empezar
  con `wide`; el modelo pinhole debe validarse especialmente en los bordes.
- `image_coordinates`: exactamente `native_unrotated`. Píxel `u` hacia la derecha,
  `v` hacia abajo; origen en el píxel superior izquierdo de la imagen guardada.
  Seleccionar sobre una vista rotada exige convertir al píxel nativo antes de
  guardar; no usar coordenadas de pantalla, retrato o imagen redimensionada.
- `intrinsics_row_major`: la matriz K de **esa imagen**, no del preview ni de otra
  resolución. El grabador iOS ya exporta este campo por JPEG. `width` y `height`
  deben coincidir con él. El modelo admite focales distintas por toma y no skew.
- `xyz_lidar_m`: metros en el sistema original del LiDAR. No aplicar el perfil
  preliminar antes de guardar. `uv_px` debe identificar el mismo punto físico.
- `id`: identificador único de vista y, si se proporciona, de pareja en esa vista.
  Los identificadores de pareja son locales: se pueden repetir entre vistas.
- `split`: exactamente `train` o `validation`. Todos los pares de validación se
  evalúan; no se eliminan errores de validación para mejorar las estadísticas.
- `static_pair_confirmed`: debe ser `true` tras confirmar el operador que nube y
  JPEG corresponden a la misma colocación inmóvil. Un campo ausente o `false`
  bloquea el ajuste. La afirmación del operador no es una sincronización medida.
- `image` es informativo para revisión. El solver no abre imágenes; calcula a
  partir de las coordenadas y K. Puede conservarse metadata adicional de la toma.
  `image_sha256`, `cloud_source`, `lidar_interval` y `camera_timestamp_s` se pueden
  conservar como procedencia en la entrada; no se usan para ajustar el reloj.

## Cálculo y lectura del resultado

La inicialización usa `solvePnPRansac` sobre coordenadas de imagen normalizadas
con la K de cada vista. El refinamiento Levenberg–Marquardt minimiza residuales
en píxeles con cada K original. RANSAC y el reajuste de consenso sólo usan `train`.
El umbral normalizado usa la focal más grande del conjunto para ser conservador
respecto al umbral en píxeles.

OpenCV devuelve cámara `+X` derecha, `+Y` abajo, `+Z` adelante. ATLAS/ARKit necesita
`+X` derecha, `+Y` arriba, `−Z` adelante. Se convierte mediante:

```text
S = diag(1, -1, -1)
R_arkit = S * R_opencv
t_arkit = S * t_opencv
p_camera_arkit = R_arkit * p_lidar_m + t_arkit
centro_camara_en_lidar = -R_arkit^T * t_arkit
```

El resultado incluye `lidar_to_camera`, el transform OpenCV para auditar,
`camera_center_lidar_m`, errores por vista y pareja, máscaras de consenso de
ajuste, estadísticas de todos los pares de entrenamiento y de los pares usados,
y estadísticas de **todos** los pares independientes de validación. Cada pareja
guarda el píxel predicho y el vector residual para dibujar el overlay y detectar
sesgos en distintos lados de la imagen.

Filtros numéricos del prototipo:

| Comprobación | Criterio |
| --- | --- |
| Datos mínimos | 3 vistas train + 2 validation; ≥4 pares por vista |
| Extensión por vista | SVD 3D: primera singular ≥0.05 m y segunda/primera ≥0.01; imagen: primera ≥10 px y segunda/primera ≥0.01 |
| Geometría de ajuste | SVD centrada de todos los puntos: tercera/primera ≥0.005; se vuelve a comprobar tras rechazar outliers |
| Consenso | ≥65% de pares train y ≥3 pares/≥50% en cada vista train |
| Error de consenso | p95 de los pares empleados ≤umbral RANSAC en píxeles |
| Observabilidad local | Condición del jacobiano con columnas de traslación escaladas por la extensión de escena <10⁶ |
| Profundidad | Todos los puntos delante de cámara, a más de 0.05 m |
| Traslación del montaje | ≤1 m; modificable con `--max-translation-m` |
| Validación independiente | Mediana ≤3 px, p95 ≤5 px, máximo ≤10 px |

`--ransac-px` modifica el umbral de consenso inicial (4 px). El p95 aceptable de
validación puede fijarse con `--max-validation-p95-px`. Son umbrales de revisión
del proyecto, **no una equivalencia a exactitud milimétrica**. Valores pequeños
no prueban por sí solos que el montaje, las unidades o los vértices sean correctos.

Antes de aceptar un perfil hay que revisar la reproyección sobre capturas que no
participaron en el ajuste, comprobar la ubicación física de la cámara y confirmar
el comportamiento cerca/lejos y en distintas zonas de imagen. Residuales que
crecen en los bordes pueden indicar distorsión no modelada, correspondencias
mal seleccionadas o intrínsecos/resolución incorrectos. No compensarlos inventando
una traslación del soporte.

La sincronización temporal es una medición distinta. Este solver no estima el
reloj, ni `encoded_unix_s` ni la hora de llegada por Wi-Fi prueban sincronización.
El perfil espacial propuesto no habilita por sí solo la evidencia RGB del filtro.

## Estimación temporal con movimiento medido

`estimate_time_offset.py` compara la magnitud de la velocidad angular obtenida de
las poses ARKit con la magnitud del giroscopio del MID-360. Se necesita una nueva
captura con registro de poses a frecuencia completa: las poses guardadas junto a
los JPEG de 5 fps de las versiones anteriores no son el registro requerido.

Con ambos sensores rígidos, grabar un segmento de giros variados y pausas,
sin sacudidas bruscas ni movimientos repetitivos idénticos. El estimador exige
al menos 12 segundos, suficiente variación angular y tracking ARKit normal;
reserva el 35% final para comprobar el desfase calculado con el primer 65%.

```sh
build/calibration-venv/bin/python tools/calibration/estimate_time_offset.py \
  --poses output/calibracion/iphone \
  --gyro output/calibracion/intervalo-movimiento/gyro.csv \
  --output output/calibracion/propuesta-reloj.json
```

`--poses` acepta la carpeta de cámara o su `poses.jsonl`. Cada línea contiene
`schema_version: 1`, `sequence` consecutiva, `timestamp_ns` entero, `tracking_normal`
booleano y `world_from_camera_row_major` con 16 valores. Su carpeta debe contener
`complete.json` con `complete: true`, `pose_log_complete: true`, `pose_samples`
coincidente y `pose_samples_dropped: 0`. El CSV usa `timestamp_ns`, `clock_source`
y `gyro_x_rad_s`, `gyro_y_rad_s`, `gyro_z_rad_s`, como exporta la herramienta C++.

La búsqueda predeterminada abarca ±15 segundos respecto a los inicios de los
registros (`--search-seconds`), con paso de 2 ms (`--step-ms`). El desfase absoluto
se reconstruye restando anclas enteras; el tamaño del paso **no representa la
exactitud**. Capturas quietas, periódicas, incompletas o con discrepancia entre
segmentos se rechazan. Código `0` significa candidato para revisión y `2`, rechazo.

La incertidumbre que se informa es empírica del ajuste de movimiento; no es una
cota demostrada del error de reloj, latencia de exposición o rolling shutter.
La salida siempre mantiene `clock_alignment_verified: false` y
`automatic_activation_allowed: false`. No certifica automáticamente el requisito
de 15 ms del filtro RGB ni sustituye una comprobación independiente en movimiento.

Referencias primarias: [OpenCV PnP](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html),
[calibración cámara–LiDAR de Livox](https://github.com/Livox-SDK/livox_camera_lidar_calibration),
[intrínsecos ARKit](https://developer.apple.com/documentation/arkit/arcamera/intrinsics).
El manual Livox muestra el método de esquinas; declara pruebas en Mid-40,
Horizon y Tele-15, no una integración lista para MID-360/ATLAS/macOS.
