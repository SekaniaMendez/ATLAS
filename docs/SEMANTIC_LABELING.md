# Etiquetar estructura y objetos para ATLAS

La herramienta usa las mediciones guardadas en `output/`. Propone suelo, paredes
y techo a partir de la geometría y la gravedad registrada por el SLAM, y agrupa
los puntos restantes como posibles objetos. Puedes revisar estos grupos y guardar
etiquetas humanas para un futuro modelo propio.

No inicia el MID-360, la cámara ni un entrenamiento. Las categorías de muebles,
como silla o mesa, las asigna la persona que revisa: todavía no hay una red que
las reconozca automáticamente.

## Abrir las capturas existentes

Desde la carpeta del proyecto en la Mac:

```sh
./startLabeling.sh
```

Abre `http://127.0.0.1:8770/` en la Mac. El selector superior permite cambiar de
captura. La primera ejecución prepara los grupos; las siguientes reutilizan los
datos si coinciden la fuente, la configuración y sus comprobaciones de integridad.

Para abrir el editor desde ATLASViewer en iPhone o iPad:

```sh
./startLabeling.sh --lan
```

El servidor muestra enlaces de emparejamiento con la dirección de la Mac y un
fragmento `#token=…`. Deja el comando abierto mientras etiquetas. `Ctrl+C` cierra
el servidor y conserva las etiquetas ya guardadas. Sin `--lan`, sólo acepta
conexiones desde la propia Mac.

Si faltan las dependencias, ejecuta `./scripts/setupGeometry.sh` como indica el
lanzador. Otros usos:

```sh
# Preparar todas las capturas sin abrir un servidor.
./startLabeling.sh --prepare-only

# Trabajar con una sola captura.
./startLabeling.sh --input output/20261028-172733/mapa.ply --lan

# Crear un conjunto nuevo sin sustituir etiquetas anteriores.
./startLabeling.sh --input output/20261028-172733/mapa.ply \
  --destination output/20261028-172733/anotaciones-alternativas --prepare-only

# Elegir otro puerto si 8770 ya está ocupado.
./startLabeling.sh --lan --port 8771
```

`--destination` requiere una sola entrada. Si una captura o la configuración han
cambiado, el programa rechaza reutilizar un conjunto incompatible en lugar de
reemplazar sus etiquetas.

## Usarlo dentro del iPhone o iPad

La app tiene dos espacios: **Mapa** muestra el SLAM y sus superficies en vivo;
**Etiquetar** abre las capturas preparadas que sirve la Mac. El mapa usa TCP
`47777`; las etiquetas usan HTTP `8770` de forma predeterminada.

1. Conecta la Mac y el dispositivo a la misma red y ejecuta `./startLabeling.sh --lan`.
2. En ATLASViewer entra a **Etiquetar** y pulsa **Conectar con la Mac**.
3. Pega el enlace completo que imprimió el servidor y pulsa **Conectar**.
4. Selecciona la captura y comienza la revisión.

Bonjour anuncia `_atlas-labels._tcp` y permite encontrar la dirección en **Macs en
la red**. El anuncio no contiene la clave: para el primer emparejamiento hay que
pegar el enlace completo. La app guarda la clave en el llavero del dispositivo y
sólo guarda la dirección sin clave en sus preferencias. El enlace permite acceder
a estas capturas; no debe publicarse junto con el código.

La cámara permanece apagada hasta que la actives expresamente en **Mapa → Ajustes
→ Cámara**. Ver el mapa o etiquetar no necesita cámara. En **Mapa**, los controles
principales son el mapa, el estado y las capas; conexión, calibración y grabación están
en **Ajustes**. **Desconectar** permite detener la conexión desde los ajustes sin
borrar las capturas ni las etiquetas guardadas.

### Internet compartido del teléfono

Puedes intentar conectar la Mac al internet compartido del iPhone y usar la
dirección de la Mac en esa conexión. Copia el enlace de la interfaz correspondiente
entre los que muestra el lanzador. Usa **la IP de la Mac**, no la dirección del
teléfono que actúa como puerta de enlace.

La interfaz Ethernet del MID-360 es una red distinta. No confundas una dirección
de esa red, como `192.168.1.1`, con la dirección que el iPhone puede alcanzar por
Wi-Fi o internet compartido. Si cambia la red o la IP de la Mac, vuelve a pegar
el enlace adecuado en el botón de conexión de **Etiquetar**. Si Bonjour no aparece,
el enlace manual sigue disponible.

Este flujo está contemplado en la configuración, pero todavía no se ha validado
con una prueba física completa de internet compartido. USB permite instalar la
app; no debe asumirse que crea por sí solo un túnel al servidor.

### Instalación y primera apertura en los dispositivos

La vista y el editor se adaptan a iPhone e iPad. Para instalar una compilación de
desarrollo desde Xcode, el dispositivo debe estar conectado, desbloqueado y tener
activado **Ajustes → Privacidad y seguridad → Modo de desarrollador**. Puede ser
necesario reiniciar y confirmar la activación.

El 29 de octubre de 2026, el Modo de desarrollador ya estaba activado y la nueva
app universal se compiló e instaló correctamente en el **iPhone 14 Pro** y el
**iPad Air de quinta generación con M1**, después del registro autorizado del
dispositivo y la actualización del perfil. El iPad se abrió y se emparejó con el
servidor de la Mac; el usuario confirmó que ve la captura y la lista de grupos.
La apertura remota del iPhone quedó pendiente de desbloquearlo.

Si aparece ese aviso, abre **Ajustes → General → VPN y gestión de dispositivos**,
selecciona el perfil correspondiente y pulsa **Confiar**, siguiendo la confirmación
del sistema. Después abre ATLASViewer y permite el acceso a la red local.

## Revisar y etiquetar

- El visor 3D permanece en su sitio mientras desplazas la lista de grupos y los
  controles. Las capas separan suelo, paredes, techo, objetos y puntos sin agrupar.
  El techo comienza oculto para inspeccionar el interior; puedes mostrarlo cuando
  necesites revisarlo. Ocultar una capa sólo cambia la vista: no borra ni ignora
  sus puntos y no cambia las etiquetas guardadas.
- Arrastra para girar y usa rueda o pellizco para acercarte. En la Mac, el botón
  derecho desplaza la vista. **Vista superior**, **Vista frontal** y **Encuadrar** ayudan a
  comprobar la estructura.
- Selecciona un grupo en la nube o en su fila. **Selección** y **Aislar
  selección** facilitan inspeccionar sus puntos y dimensiones.
- Los filtros separan estructura, posibles objetos, pendientes, etiquetados y
  puntos sin agrupar. Las propuestas geométricas aparecen como propuestas.
- Comprueba forma y posición antes de confirmar una propuesta con **Confirmar
  suelo**, **Confirmar pared** o **Confirmar techo**. También puedes cambiar la
  categoría manualmente.
- Para un objeto, elige una categoría, añade una nota si hace falta y pulsa
  **Guardar etiqueta**. Seleccionar o cambiar el desplegable no guarda nada por sí solo.
- Si varios grupos son fragmentos del mismo objeto, selecciónalos con las
  casillas o con `Ctrl`/`⌘`/`Shift` + clic y pulsa **Etiquetar grupos juntos**.
  Conservan sus grupos originales, pero comparten una instancia humana.
- **Ignorar selección** la excluye de los ejemplos etiquetados. **Volver a
  pendiente** retira la etiqueta para revisarla después. Ninguna opción elimina puntos.

El catálogo inicial incluye suelo, pared, techo, silla, mesa, sofá, armario,
puerta, ventana y otro. La nota permite explicar una observación incompleta o
describir el ejemplar concreto.

Un grupo geométrico puede contener parte de un objeto o varios objetos que se
tocan. Puedes reunir grupos completos mediante una instancia o extraer puntos
individuales para crear un objeto nuevo. El grupo sin asignar puede mezclar
regiones sin relación entre sí; conviene seleccionar sólo los puntos que puedas
identificar antes de usarlos como ejemplo.

### Crear categorías propias

Junto al selector **Categoría**, pulsa **Nueva categoría**. Puedes hacerlo aunque
todavía no hayas seleccionado un grupo. Escribe un nombre de hasta 80 caracteres,
por ejemplo «Lámpara», y pulsa **Crear categoría**; **Cancelar** cierra el formulario.
Al abrirlo se actualiza el catálogo para incluir categorías creadas desde otro
dispositivo.

La categoría queda seleccionada, pero **no etiqueta ni guarda puntos**. Revisa el
grupo o el borrador y pulsa después **Guardar etiqueta** o **Crear objeto
etiquetado**. El borrador, las notas y las capas se conservan al crear la categoría
o actualizar la lista. Si el nombre ya existe, se selecciona esa misma categoría;
las diferencias de mayúsculas y los espacios sobrantes no crean otra clase.

Las categorías se comparten entre las capturas servidas con el mismo catálogo.
Los identificadores iniciales `0–9` permanecen fijos; las categorías propias
reciben identificadores estables desde `10`. Una «Lámpara» conserva el mismo ID y
color al etiquetar otra captura. Las clases propias se muestran en la capa
**Objetos**. Crear nombres y ejemplos prepara los datos: no entrena una red ni
hace que ATLAS reconozca esa categoría automáticamente.

El lanzador guarda por defecto el catálogo en `output/categories.json` y su
archivo oculto de identidad y comprobación en `output/.categories.json.identity`.
Conserva y respalda **ambos archivos juntos**, además de las carpetas de
anotaciones. Para una copia consistente, detén antes el servidor con `Ctrl+C`.
El archivo `output/.categories.json.lock` coordina el acceso; no contiene las
categorías. Si usas `--taxonomy` para elegir otra ruta, su archivo de identidad
se guarda al lado con el mismo patrón de nombre.

No renumeres ni sustituyas estos archivos para cambiar una clase: el catálogo
incluye comprobaciones para rechazar pérdidas o cambios de significado en IDs ya
asignados. Esta versión permite añadir categorías; no permite renombrarlas ni
eliminarlas. Si aparece un error de catálogo, restaura el catálogo y su archivo de
identidad de una misma copia. Los IDs de dos catálogos independientes no se deben
considerar equivalentes sólo porque tengan el mismo número.

### Crear un objeto seleccionando puntos

En iPad, el Apple Pencil pinta la selección de puntos medidos y los dedos siguen
sirviendo para girar, acercar y desplazar el mapa. Activa **Pintar puntos** para
usar ratón o toque sin Pencil; vuelve a **Girar / elegir grupo** para navegar con
ellos. **Radio** cambia el alcance del pincel en pantalla, sin modificar el tamaño
ni la posición de los puntos.

Sólo se seleccionan puntos dibujados visibles; los que quedan tapados por otros
puntos no se incluyen. Los huecos reales de la nube siguen siendo visibles: el
pincel no supone que una pared dispersa sea una superficie cerrada. Las capas
ocultas quedan fuera de la selección. Gira el objeto y añade trazos desde otras
vistas para completar las superficies que sí se midieron. La herramienta no
reconstruye partes que no aparecen en la captura.

**Borrar selección** permite quitar puntos con el pincel; **Deshacer trazo**
revierte el último trazo y **Cancelar selección** limpia el borrador. Después
elige una categoría y pulsa **Crear objeto etiquetado**: los puntos seleccionados
se separan de sus grupos anteriores y pasan a un grupo nuevo con una instancia y
etiqueta humanas. La selección puede tomar puntos de varios grupos e incluso de
objetos manuales anteriores; éstos conservan sus puntos y etiquetas restantes.
Cada creación admite hasta 100.000 puntos.

**Deshacer objeto manual → Devolver puntos** devuelve **sus puntos actuales** a
los grupos base y recupera las etiquetas que esos grupos conservan. No elimina
mediciones ni revierte puntos que ya transferiste a otro objeto manual. No es lo
mismo que deshacer un trazo todavía sin guardar, ni que volver una etiqueta a
pendiente. Los grupos que quedan sin puntos desaparecen de la lista; los grupos
base pueden reaparecer al restaurar sus puntos.

Crear o eliminar estos objetos modifica sólo `annotations.json`. `manifest.json`
y `points.npz` mantienen los grupos base, el orden canónico y las coordenadas
originales. No se desplazan, duplican ni inventan puntos.

Los guardados llevan número de revisión. Si otra ventana modifica el mismo
conjunto, el editor pide recargar antes de guardar; no sustituye silenciosamente
cambios ajenos. **Recargar etiquetas guardadas** conserva el borrador de puntos
para revisarlo con la partición actual. Cambiar de captura con puntos sin guardar
pide confirmar que quieres descartar ese borrador.

## Exportar ejemplos de entrenamiento

Pulsa **Exportar datos**. Se crea una exportación de la revisión
actual y aparecen enlaces para descargar NPZ y JSON. En la app, la descarga puede
abrirse en la hoja para compartir. Aunque no descargues una copia al dispositivo,
los archivos quedan guardados en la Mac:

```text
output/<captura>/anotaciones/
  manifest.json                     Fuente, grupos y propuestas geométricas
  points.npz                        Coordenadas y grupos base inmutables
  annotations.json                  Etiquetas, objetos manuales y revisión actual
  training/revision-000001/
    data.npz                        Puntos y objetivos de entrenamiento
    metadata.json                   Clases, procedencia, revisión y hashes
```

Los nombres de descarga son `revision-000001.npz` y `revision-000001.json`; en la
carpeta de revisión se llaman `data.npz` y `metadata.json`.

El NPZ contiene `points`, `point_group_ids`, `base_point_group_ids`, `canonical_point_indices`,
`semantic_labels`, `instance_ids`, `reviewed_mask` e `ignored_mask`. Son coordenadas
medidas, en metros y en el mapa SLAM con Z hacia arriba, no vértices inventados de
una malla. La copia de trabajo usa XYZ únicos ordenados lexicográficamente y
Float32. Sus índices canónicos no equivalen al número de fila del PLY original.

`point_group_ids` describe la partición efectiva de esa revisión, incluidos los
objetos creados seleccionando puntos. Sus índices corresponden a la lista
`groups` del JSON exportado. `base_point_group_ids` conserva la pertenencia
original y se interpreta con `base_group_ids`. El JSON también incluye
`manual_groups`, con los índices canónicos asignados manualmente y su procedencia
`human_selection`. Los identificadores de grupo se conservan aunque su posición
en la lista efectiva cambie al crear o eliminar objetos. Las exportaciones de
revisiones anteriores permanecen intactas.

Sólo las etiquetas humanas guardadas se convierten en objetivos. Los puntos
pendientes o ignorados conservan `semantic_labels = -1` e `instance_ids = -1`.
`ignored_mask` distingue lo ignorado de lo pendiente. Una propuesta automática de
suelo o pared no se convierte en una etiqueta verdadera sin confirmación.

El JSON incluye la taxonomía completa usada al exportar, la identidad y revisión
del catálogo (`taxonomy_catalog_id`, `taxonomy_revision`), procedencia y hashes.
Los IDs de clase se comparten entre capturas del mismo catálogo, incluidas las
categorías propias. Los identificadores de instancia son locales a la captura;
no garantizan identificar el mismo objeto en otro recorrido.

Cada exportación conserva una copia fija del catálogo de ese momento. Añadir
categorías después no modifica sus archivos; volver a exportar la misma revisión
devuelve la exportación existente. Para interpretar los objetivos del NPZ, usa
siempre la taxonomía de su JSON correspondiente.

**Exportar no inicia un entrenamiento.** El reparto queda como
`split = "unassigned"`, con `split_group` basado en la captura y `location_id`
vacío. Antes de entrenar habrá que identificar los lugares y mantener juntos
todos los recorridos del mismo lugar al dividir entrenamiento y validación.
Separar al azar puntos de una misma habitación produciría una evaluación engañosa.

## Conservación y resultados iniciales

Los PLY originales y las geometrías existentes no se modifican ni eliminan.
Preparar, etiquetar y exportar escriben sus propios archivos bajo `anotaciones/`;
el catálogo compartido y su archivo de identidad se guardan en `output/`.
Para nuevas sesiones que quieras usar como datos de entrenamiento:

```sh
./startAtlas.sh --keep-source
```

Un GLB simplificado puede perder detalles útiles para reconocer objetos. La
captura destinada a entrenamiento necesita sus puntos, además del modelo ligero
destinado a visualizar o compartir.

| Captura | Puntos conservados | Regiones estructurales | Posibles objetos | Grupo sin asignar |
| --- | ---: | ---: | ---: | ---: |
| `20261028-172542` | 61.571 | 13 | 25 | 1 |
| `20261028-172733` | 24.184 | 7 | 27 | 1 |

Estos números cuentan propuestas geométricas, no objetos reconocidos por IA ni
etiquetas ya confirmadas. Si falta soporte o una referencia válida de gravedad,
se mantiene la incertidumbre sobre la estructura.

La etapa complementa la [reconstrucción de geometría](STRUCTURAL_GEOMETRY.md).
Después de revisar ejemplos variados podremos entrenar y evaluar una red pequeña
con capturas de lugares que no haya visto antes.

## Comprobación de capas, puntos y categorías

```sh
OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 build/structure-venv/bin/python -m unittest discover -s tests/structure -v
node --test tests/structure/test_annotation_selection.cjs
```

La actualización se comprobó con 88 pruebas Python y 11 pruebas de selección
en JavaScript.
En una copia de la captura se comprobó pintar 711 puntos, borrar parte, deshacer,
crear el objeto, recargarlo, exportar exactamente esos puntos y devolverlos a sus
grupos base. Las capturas reales no se etiquetaron durante estas pruebas. Se
comprobó la vista en anchos de 393 y 1180 píxeles, con el techo opcional y el visor
visible mientras se desplazan las herramientas. La actualización nativa compiló
e instaló en el iPad. El 29 de octubre de 2026, el usuario confirmó que puede
seleccionar puntos con el Apple Pencil en el iPad Air M1.

El flujo de categorías también se comprobó de extremo a extremo en una copia de
prueba: se pintaron 711 puntos y se creó «Lámpara de pie» con ID `10`, conservando
la nota y la revisión `0` de la captura. Repetir el nombre con otras mayúsculas y
espacios reutilizó esa categoría, disponible en las dos capturas. Guardar el
objeto avanzó a revisión `1`; su exportación contenía exactamente 711 objetivos
`int16` con ID `10` y `-1` en los puntos restantes. Estas pruebas no modificaron
las categorías ni las etiquetas de las capturas de producción.
