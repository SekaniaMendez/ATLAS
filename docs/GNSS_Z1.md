# Entrada GNSS del SingularXYZ Z1

ATLAS puede registrar las posiciones del Z1 por la consulta de lectura que usa
su panel web. Se verificó con un Z1 en modo **Rover**, firmware **V4.2.0.3** y
firmware GNSS **R4.10Build14208**, conectado por Wi-Fi a la Mac. Esta primera
etapa recibe y conserva observaciones; todavía no aplica anclas a la nube LiDAR.

## Conectar y registrar

1. Conecta la Mac al Wi-Fi del Z1. En el equipo probado, el receptor está en
   `192.168.10.12`; la dirección de la Mac la asigna esa red.
2. Abre `http://192.168.10.12/` y comprueba el modo Rover y la solución. El estado
   observado `NARROW_INT` corresponde al tipo de solución RTK con ambigüedades
   fijadas; también se validan estado de cálculo, tiempos y edad de correcciones.
3. Desde la raíz de ATLAS:

   ```sh
   ./startGnss.sh --seconds 60
   ```

   Para registrar hasta pulsar Ctrl+C:

   ```sh
   ./startGnss.sh --seconds 0
   ```

   Si cambia la dirección, usa `--host IPv4`. `--output CARPETA_NUEVA` permite
   elegir un destino. Nunca se sobrescribe un registro existente. No se necesita
   instalar paquetes Python adicionales.

El comando consulta `device.cgi`, `status.cgi` y `position.cgi` mediante POST
vacío, igual que el panel del firmware probado. No envía comandos de montaje,
base, rover, reinicio ni salida COM. No instala un cliente NTRIP: utiliza las
posiciones que el receptor ya está calculando con sus correcciones actuales.
Es una interfaz del firmware observado; no se declara una API universal del Z1.

La pantalla **COM 1 → NMEA-0183** configura una salida serie. Su número de puerto
no es un puerto TCP. No hace falta activar esa salida para esta lectura Wi-Fi.
La ficha del producto declara NMEA y TCP/UDP, pero este panel no ha confirmado
una salida NMEA TCP configurable. El registrador identifica su transporte como
`Z1 WebUI HTTP polling` para no confundirlo con un flujo NMEA directo.

El manual V1.0 del Z1 indica que Wi-Fi y Bluetooth no coexisten. Si las
correcciones llegan por teléfono/Bluetooth, revisa el estado RTK después de
activar Wi-Fi. En la prueba se observó Rover con `NARROW_INT` por Wi-Fi.
Contraseñas de fábrica y procedimiento: [manual oficial, página impresa 31](https://www.singularxyz.com/download-file/20).

## Archivos conservados

El destino predeterminado es `output/gnss/FECHA_UTC-identificador/`. La fecha lleva
`Z` porque está en UTC; puede diferir del día local de Costa Rica.

- `raw.jsonl`: cada respuesta original y su resultado de validación, con inicio
  de consulta, recepción monotónica y recepción Unix en la Mac. Conserva también
  respuestas repetidas, rechazadas y errores de consulta.
- `fixes.csv`: una fila por época GPS nueva, con posición, altura, datum declarado,
  incertidumbre, calidad, edad de correcciones, tiempo GPS/UTC y tiempos de
  recepción. Las épocas repetidas o anteriores se excluyen del CSV y quedan en
  el registro crudo. Una fila no implica que sea apta como ancla: revisar
  `position_valid`, `time_valid`, `epoch_coherent` y `quality_usable`.
- `session.json`: equipo, firmware, modo, conteos, motivo de finalización y las
  verificaciones pendientes. Un fallo o una sesión sin épocas válidas no se
  presenta como una captura completa. Una sesión con errores queda `partial`.

El endpoint probado entrega `BESTPOSA`, `TIMEA`, `GPRMC` y `SATINFO`. ATLAS valida
CRC32 de BESTPOSA/TIMEA y su estructura compatible con el formato ASCII NovAtel.
Comprueba las épocas GPS, coherencia del calendario UTC, estado del reloj,
coordenadas finitas y válidas, satélites e incertidumbres. Rechaza como utilizables
los RTK con correcciones de más de cinco segundos. Conserva FIX/FLOAT y calidad
por separado: una solución declarada FIX puede fallar las demás comprobaciones.

Los valores sigma son estimaciones del receptor, no una validación independiente
de exactitud. `quality_usable` sólo indica coherencia de esta observación;
todavía faltan los controles físicos, temporales y geodésicos para usarla en SLAM.
La altura reportada y la ondulación se conservan por separado. La suma calculada
según BESTPOS no verifica el modelo geoidal configurado, la referencia física de
antena ni el datum de la base. Esas verificaciones siguen marcadas como falsas.

También existe `tools/gnss/nmea.py`, lector validado de GGA, RMC, ZDA y GST para
una futura salida NMEA directa. Distingue altura MSL, separación geoidal y altura
elipsoidal; no inventa una fecha a partir de un GGA sin fecha.

## El GNSS como punto ancla del lienzo

Las observaciones quedan identificadas con el rol `gnss_control_observation`.
Una vez calibradas, alimentarán una capa de puntos de control: coordenadas,
tiempo, calidad y vínculo con una pose del recorrido. Los puntos que describen
superficies mantienen su función geométrica; el ancla establece restricciones
sobre dónde está el conjunto de sensores en el mundo real.

Un primer FIX estable puede establecer el origen de un marco local ENU
(Este, Norte, Arriba). Una posición aislada no determina el rumbo. Varias
posiciones durante un recorrido suficientemente amplio, o referencias de
orientación independientes, permiten orientar el mapa. La escala métrica del
LiDAR debe conservarse. La precisión de cada observación determina su peso;
ningún punto adquiere confianza infinita por llamarse RTK.

El modelo del montaje será:

```text
antena_ENU = R_ENU_mapa × (posición_LiDAR_mapa + R_mapa_LiDAR × brazo_LiDAR_antena)
            + traslación_ENU_mapa
```

El brazo es un vector tridimensional medido desde el origen LiDAR a la referencia
GNSS, expresado en los ejes del LiDAR. La rotación de la pose compensa la
inclinación del bastón. También hay que confirmar si la salida del receptor
describe la antena o una punta de bastón ya compensada, para evitar compensarla
dos veces. El rig de Blender contiene una reserva GNSS; sus dimensiones de
maqueta no son esta calibración medida.

**Pendiente antes de aplicar anclas:** sincronizar el reloj LiDAR con el tiempo
GNSS, medir el montaje y confirmar datum/altura/referencia física. El tiempo de
llegada HTTP no sustituye al instante GNSS; el registrador conserva ambos.
Por ahora `clock_alignment_verified`, `lever_arm_verified` y
`georeferenced_lidar` permanecen en falso.

Una transformación rígida ubica y orienta toda la nube. Corregir su deriva
interna requiere ajustar la trayectoria con las anclas y reconstruir los puntos
a partir de escaneos o keyframes conservados. El PLY agregado actual no conserva
la procedencia temporal de cada punto; `startAtlas.sh --keep-source` conserva
PLY/mapa, pero aún no activa la grabación cruda `--record` del probe. Las capturas
antiguas de `output/` no contienen un registro GNSS simultáneo.

## Pruebas

```sh
python3 -m unittest discover -s tests/gnss -v
```

Las pruebas usan coordenadas sintéticas. Las lecturas reales quedan sólo en
`build/` u `output/`, excluidos de Git. La adquisición por Wi-Fi y su calidad se
comprueban con el registro de cada sesión; estas pruebas no certifican todavía
una nube georreferenciada ni precisión topográfica del sistema completo.

El 29 de octubre de 2026 se completaron 49 pruebas automatizadas y un registro
real de un minuto: 60 épocas distintas, 10 RTK FIX, 47 RTK FLOAT y 3 DGNSS, sin
errores de lectura. La consulta HTTP tardó aproximadamente 0,82–1,02 segundos;
ese tiempo de ida y vuelta no equivale a una sincronización de sensores ni a la
edad exacta de la posición. El registro fue pasivo y no cambió la configuración
del receptor.

Referencias del formato: [ASCII y CRC32](https://docs.novatel.com/OEM7/Content/Messages/32_Bit_CRC.htm),
[BESTPOS](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm),
[TIME](https://docs.novatel.com/OEM7/Content/Logs/TIME.htm).
