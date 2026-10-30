# Geometría estructural y marcos de referencia

ATLAS convierte un mapa confirmado del SLAM en superficies observadas: planos
acotados con un marco local propio y una malla para los detalles no planos.
El resultado principal es `scene.glb`, importable en Blender, acompañado de
`scene.json` con ecuaciones, transformaciones, áreas y procedencia. No contiene
una nube de puntos. Los colores distinguen superficies; no son fotografías ni
texturas de la cámara del teléfono.

La reconstrucción detallada funciona **al terminar la captura**, también sin sensores usando un
PLY existente. Durante el escaneo hay además una vista previa nativa de planos y
ejes en el iPhone o iPad, descrita abajo. No cambia la estimación del SLAM.
No reconoce semánticamente habitaciones, puertas o muebles,
no georreferencia y no promete un sólido cerrado: las zonas sin mediciones
permanecen desconocidas.

## Uso

El entorno de esta Mac está preparado en `build/structure-venv`. Para instalarlo
en otro equipo se necesita Python 3.12:

```sh
./scripts/setupGeometry.sh
# Si Python no está en PATH:
ATLAS_GEOMETRY_PYTHON=/ruta/python3.12 ./scripts/setupGeometry.sh
```

Reconstruir una captura sin conectar el Livox:

```sh
./createGeometry.sh output/20261028-172733/mapa.ply
# Opcional: elegir un directorio que todavía no exista
./createGeometry.sh output/20261028-172733/mapa.ply output/mi-geometria
```

El comando conserva el original y rechaza sobrescribir una salida existente.
Lee los metadatos de `slam_session.txt` junto al PLY o dentro de `mapa/` o `map/`.
Sin esos metadatos todavía exporta superficies, pero no declara un marco alineado
con gravedad ni autoriza retirar la fuente. Admite PLY ASCII de ATLAS con
coordenadas finitas y hasta dos millones de puntos; no es un lector PLY general.

Para nuevas capturas:

```sh
./startAtlas.sh
# Capturar con el MID-360 quieto al inicio; Ctrl+C termina y reconstruye.
# Para conservar siempre mediciones y archivo de voxels:
./startAtlas.sh --keep-source
```

Salidas dentro de `output/SESION/geometria/`:

| Archivo | Contenido |
| --- | --- |
| `scene.glb` | Malla binaria con un objeto por plano y objetos de detalle |
| `scene.json` | Planos, marcos, unidades, áreas, soporte e identificación de la fuente |
| `quality.json` | Cobertura, distancias, tamaños y decisión de conservar el respaldo |
| `viewer.html` | Visor autónomo sin internet: giro, zoom, selección y ejes |

Abre `viewer.html` en Chrome/Safari o importa `scene.glb` en Blender. El visor
duplica los buffers de geometría para funcionar por sí solo; para compartir o
archivar el modelo basta el GLB y sus JSON, sin el visor.

## Extracción y referencia espacial

1. Se detectan planos con RANSAC y normales locales, y se refinan con sus puntos
   de soporte. Las regiones coplanares desconectadas se separan.
2. Se construye un contorno a partir de triángulos locales y se simplifica dentro
   de su soporte. Los huecos observables se conservan; no se utiliza una envolvente
   convexa de toda la pared que rellene automáticamente puertas o ventanas.
3. Cada plano obtiene un origen en su soporte y ejes ortonormales: X/Y tangentes
   y Z normal. `T_parent_local_rows` transforma del plano al mapa SLAM.
4. Si el SLAM documentó gravedad, se deriva un marco estructural con Z vertical
   y una dirección horizontal a partir de un plano vertical dominante. Sin
   orientación horizontal observable se conserva el yaw del mapa y se declara
   esa limitación. No se fuerzan todas las paredes a ser ortogonales.
5. Los detalles no explicados por los planos se reconstruyen con triangulación
   local y simplificación, y se valida la malla completa contra las mediciones.

Las ecuaciones son `normal · punto_mapa + offset = 0`, en metros. El mapa original
es Z-arriba. El GLB usa un nodo raíz Rx(-90°) para convertirlo al convenio Y-arriba
de glTF; no se altera la trayectoria ni el mapa SLAM. Blender revierte ese convenio
al importar. `scene.json` permanece en coordenadas ATLAS.

El origen del marco estructural es un centro de soporte, no una esquina de
levantamiento. La dirección horizontal no es norte geográfico. Los identificadores
son de una instantánea: no garantizan identidad entre recorridos diferentes.

## Parámetros y conservación del lienzo

Los parámetros están en `tools/structure/structure_config.py`: tolerancia de ajuste
de plano 2,5 cm, al menos 100 puntos y 0,12 m² de soporte, muestreo de contorno
8 cm, conexiones locales hasta 25 cm y simplificación de borde 1,5 cm. Son valores
iniciales para los mapas actuales, no una garantía de precisión del MID-360.
La reconstrucción interpola entre muestras cercanas: huecos menores que esa
resolución pueden no distinguirse. Puntos aislados sin vecinos no crean superficies.
Los detalles libres usan varias pasadas locales con aristas de hasta 20 cm y
soporte de mediciones dentro de 10 cm. Su simplificación se comprueba en ambos
sentidos con vértices, centros de arista y centroides de cada triángulo: se limita
la desviación muestreada a 1,5 cm y el cambio de área al 1 %, y se restauran
triángulos o se revierte la reducción si pierde cobertura o soporte. Esto no
garantiza preservar toda la topología; los diagnósticos están en `scene.json`.

El mapa confirmado por el filtro temporal es la entrada de esta etapa. Al
reconstruir, se usan únicamente las superficies presentes en esa instantánea.
Un lugar que el lienzo borró no se rellena con una hipótesis de habitación.
La vista en vivo vuelve a extraer sus planos de la instantánea más reciente y
reemplaza el conjunto anterior, incluyendo superficies que desaparecieron. No
conserva identidades de planos entre actualizaciones. La reconstrucción detallada
se vuelve a ejecutar al guardar la captura.

## Vista en vivo en el iPhone o iPad

La app actualizada recibe superficies por la misma conexión TCP 47777 del mapa.
Ejecuta `./startAtlas.sh` y conecta ATLASViewer en **Mapa**. Cada sesión SLAM
empieza vista desde arriba. Puedes arrastrar para orbitar y pellizcar para
acercarte; el botón **Vista superior** vuelve a la vista cenital sin bloquear
los gestos posteriores. Recibir nuevas superficies no cambia continuamente el
ángulo que elegiste.

En **Capas del mapa** aparecen **Puntos**, **Superficies**, **Ejes de referencia**
y **Mostrar techo**. Los planos tienen colores distintos y marcos locales X
rojo, Y verde y Z azul. El indicador muestra cantidades de puntos y superficies.

**Mostrar techo** comienza desactivado. Cuando los planos aportan evidencia
suficiente de una superficie horizontal alta y paredes compatibles, la app
estima un corte superior en Z del mapa, unos 10 cm por debajo de esa superficie.
El mismo corte se aplica a puntos, triángulos y ejes. Así puedes inspeccionar el
interior desde arriba; activar **Mostrar techo** recupera la escena completa.

Es un **corte visual estimado**, no una etiqueta semántica de techo: también
puede ocultar partes altas de paredes u otros elementos por encima del corte.
Un plano aislado o una mesa no bastan para activarlo. Si aún falta soporte,
aparece **Buscando altura del techo…** y no se recorta la escena. La estimación
se retira al invalidarse o caducar la geometría. No se eliminan mediciones ni
se alteran el SLAM, los archivos guardados o las etiquetas de entrenamiento.
La vista de cámara/AR conserva su comportamiento propio.

La Mac calcula una vista previa cada segundo en un hilo separado, con una sola
instantánea pendiente reemplazable. No acumula trabajos si la captura avanza más
rápido. La nube continúa a aproximadamente cuatro instantáneas por segundo y
Metal dibuja la escena de forma independiente. Los valores configurables están
en `core/geometry/include/atlas/geometry/live_geometry_config.hpp` y
`ui_stream_server_config.hpp`.

La extracción C++ usa RANSAC limitado y triangulación dentro de celdas UV ocupadas
de 16 cm; necesita 80 puntos por hipótesis y 0,12 m² por superficie. La entrada
se limita a 50.000 puntos y la salida a 32 superficies, 30.000 vértices y 90.000
índices. Esta vista muestra superficies planas con soporte suficiente; los
detalles curvos o dispersos siguen visibles como puntos. La reconstrucción final
offline sigue siendo más detallada. Los colores identifican superficies, sin
añadir clasificación de objetos ni texturas RGB.

Cada mensaje reemplaza completamente los triángulos anteriores. Un mapa vacío,
una pérdida de seguimiento o una reconexión invalida también cálculos en curso.
El dispositivo retira geometría tras tres segundos sin actualizarse. Retransmitir el
mismo timestamp no renueva su antigüedad. Los huecos menores que la celda pueden
interpolarse; los IDs y los ejes se recalculan y pueden cambiar entre instantáneas.
Es una vista orbital en coordenadas SLAM; no se superpone a la cámara como AR.
Si los planos llegan antes que los puntos del mapa, el visor espera esos puntos
sin mezclar una nube anterior de sensor o demostración con la nueva geometría.

Para comprobarla sin Livox, usando una captura guardada:

```sh
./build/mac-release/apps/atlas_geometry_preview \
  --input output/20261028-172733/mapa.ply
# Conectar la app a la IP de la Mac. Ctrl+C detiene solamente esta reproducción.
```

El ejecutable no inicia el SDK del sensor y no modifica el PLY. Sus timestamps
avanzan para simular una fuente activa. `--exercise-clears` añade fases de mapa
vacío, reconstrucción y seguimiento perdido para comprobar las transiciones.

Validación adicional:

```sh
build/mac-release/tests/atlas_GeometryWire_test build/live-geometry-wire-fixture.bin
tests/ios/run_live_geometry_tests.sh build/live-geometry-wire-fixture.bin
xcrun swift -module-cache-path build/swift-geometry-module-cache \
  tests/ios/LiveGeometryMetalSmoke.swift
python3 tests/ios/test_live_geometry_stream.py --record-ply output/20261028-172733/mapa.ply
```

La prueba Metal necesita acceso a GPU; la prueba de transporte abre únicamente
sockets locales. Ninguna inicia el LiDAR ni graba cámara.

La prueba Metal verificó los cinco pipelines, el corte de puntos y ejes altos,
la conservación de la parte baja de triángulos que cruzan el corte y la
restauración de una imagen idéntica al volver a mostrar el techo. Esta prueba
usa una escena sintética en GPU; la estimación geométrica se comprueba por
separado en las pruebas del modelo.

El 29 de octubre de 2026 pasaron 11 comprobaciones Swift de protocolo/modelo,
incluidas las dos capturas de `output/`: sus cortes visuales se estimaron en
Z = 2.159 m y 2.182 m del mapa, respectivamente. Se verificó que suelo, mesa,
plano aislado y altura insuficiente no activan el corte, y que éste se limpia
con una escena vacía, una reconexión o una geometría caducada. La compilación
final se instaló y abrió en el iPad Air M1; se comprobó su conexión TCP a la
reproducción de la primera captura. El encabezado distingue esa fuente con
**Reproducción de captura**. Esa reproducción utiliza el puerto del mapa y debe
cerrarse antes de iniciar una sesión nueva con el MID-360.

## Validación antes de retirar el PLY intermedio

El lanzador solo elimina **su propio PLY recién creado**, después de que captura
y reconstrucción hayan terminado correctamente, existan los cuatro artefactos y
el hash SHA-256 de la fuente no haya cambiado. Además se exige:

- Al menos 99 % de los puntos a 4 cm o menos de la superficie reconstruida.
- Como máximo 1 % del área muestreada a más de 10 cm de una medición.
- GLB + descripción JSON al menos dos veces menores que el PLY ASCII de entrada.
- Estado final de seguimiento válido, sin error fatal, y gravedad documentada.

Si falta alguna condición, se conserva `mediciones.ply` y se informa `needs_review`.
`--keep-source` conserva el PLY incluso cuando todo pasa. La herramienta offline
nunca borra sus entradas. Estas medidas verifican una aproximación a la captura,
no su exactitud respecto al lugar real, ni garantizan conservar cada detalle del
1 % restante. La comprobación de área es estadística, no una prueba exhaustiva.

`compression_ratio` compara el PLY ASCII con GLB + `scene.json`; no compara contra
un PLY binario comprimido. `bundle_compression_ratio` incluye también el informe y
el visor portátil. El respaldo retenido y los archivos de capturas antiguas no
se cuentan como parte del modelo y siguen ocupando su espacio original.

## Base abierta y pruebas

La implementación usa [Open3D](https://github.com/isl-org/Open3D) para ajuste,
consulta de distancias y [reconstrucción de superficies](https://www.open3d.org/docs/release/tutorial/geometry/surface_reconstruction.html),
SciPy para vecindarios y [Shapely para triangulación restringida](https://shapely.readthedocs.io/en/stable/reference/shapely.constrained_delaunay_triangles.html).
Las versiones se fijan en `tools/structure/requirements.txt`.

```sh
OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 build/structure-venv/bin/python \
  -m unittest discover -s tests/structure -v
cmake --build --preset mac-release
ctest --test-dir build/mac-release --output-on-failure
# Integración completa con una captura guardada y un probe simulado, sin hardware:
build/structure-venv/bin/python tests/structure/run_recorded_e2e.py \
  --capture-dir output/20261028-172733
```

Las pruebas cubren huecos, regiones desconectadas, paredes oblicuas, marcos,
distancias bidireccionales, fallos de seguimiento, métricas inválidas y la
conservación de fuentes ante errores e interrupciones.
La integración offline envía Ctrl+C al supervisor, entrega una copia de la
captura al analizador real, verifica los artefactos y comprueba que los originales
mantienen su SHA-256. Sus resultados se guardan dentro de `build/geometry-session-e2e/`.

## Resultado sobre las capturas del 28 de octubre de 2026

Las dos capturas existentes se procesaron sin sensores y sin modificar sus
originales. Los resultados están en `output/20261028-172733/geometria/` y
`output/20261028-172542/geometria/`.

| Captura | Puntos de entrada | Planos | Cobertura a 4 cm | GLB + referencias | Reducción |
| --- | ---: | ---: | ---: | ---: | ---: |
| 17:27:33 | 24.184 | 15 | 99,83 % | 354,3 KiB | 3,34× |
| 17:25:42 | 61.571 | 30 | 99,57 % | 845,1 KiB | 3,58× |

Cada modelo incluye además una superficie libre para los detalles. Permanecen
sin representar 41 y 266 puntos, respectivamente; no se fabricaron caras para
cubrirlos. La distancia percentil 95 a las mediciones es 1,39 cm y 1,81 cm; esto
no mide exactitud topográfica. Incluyendo el visor independiente y el informe,
las carpetas son 913 KiB y 2.232 KiB (reducción de 1,29× y 1,35×). Ambos modelos
pasan los umbrales de aproximación, y los PLY antiguos permanecen intactos porque
la reconstrucción offline nunca los elimina.
