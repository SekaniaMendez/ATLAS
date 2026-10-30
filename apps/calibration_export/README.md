# Exportar una ventana RAW para calibración

Compilar desde la raíz del proyecto:

```sh
cmake --build build/mac-release --target atlas_calibration_export
```

La inspección valida toda la captura, incluyendo CRC y marcador final, e imprime un JSON sin escribir archivos:

```sh
build/mac-release/apps/atlas_calibration_export --input output/SESION/crudo --inspect
```

`seconds` resume cada segundo relativo al primer paquete LiDAR: paquetes, puntos, muestras IMU, norma RMS y máxima del gyro. Es una ayuda para localizar intervalos quietos; el gyro no detecta por sí solo traslación y **no certifica inmovilidad**. El exportador no admite capturas todavía abiertas o truncadas.

Exportar cinco segundos a una carpeta nueva cuyo directorio padre ya exista:

```sh
build/mac-release/apps/atlas_calibration_export \
  --input output/SESION/crudo --output output/SESION/calibracion-vista-01 \
  --start-s 10 --duration-s 5
```

La ventana es `[inicio, fin)`, respecto al timestamp del primer paquete LiDAR del sensor elegido. Usa el primer sensor LiDAR por defecto; `--sensor-id ID` elige otro. Cada punto se incluye según su timestamp **de paquete + offset del punto**, no por llegada a la Mac. No se mezclan otros sensores ni relojes. Un cambio de reloj LiDAR o timestamps anteriores al primero exige dividir la captura en épocas; las exportaciones con IMU de otro reloj se rechazan. La inspección cuenta retrocesos en los timestamps de paquetes de cada sensor; una exportación con retrocesos se rechaza para revisar posibles reinicios o desorden de paquetes antes de calibrar.

Los resultados son:

- `cloud.ply`: ASCII, XYZ en metros y coordenadas originales del sensor, reflectividad y tag. No contiene poses SLAM, transformación a mapa ni compensación de movimiento. Sólo una ventana con el conjunto y el objetivo inmóviles sirve para la calibración espacial.
- `points.csv`: una fila por vértice, mismo orden que el PLY, con `vertex_index,timestamp_ns,clock_source,reflectivity,tag`. Los nanosegundos son enteros decimales completos; no convertirlos primero a flotantes para restarlos.
- `gyro.csv`: `timestamp_ns,clock_source,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,accel_x_m_s2,accel_y_m_s2,accel_z_m_s2`. Incluye el mismo intervalo y sensor.
- `metadata.json`: origen temporal, ventana absoluta, sensor/reloj, contadores, filtros, índice por segundo y marco `lidar_sensor`. Mantiene explícitamente sin verificar la inmovilidad y las calibraciones espacial/temporal.
- `source_manifest.txt`: metadatos originales opacos de la captura.

Para una secuencia de giros destinada a medir el desfase temporal, añade `--imu-only`: conserva `gyro.csv`, metadatos y manifest, sin crear el PLY ni `points.csv`. Así se puede exportar toda una secuencia larga sin duplicar millones de puntos. `point_cloud_exported` indica si la salida incluye nube; `selected_points` sigue describiendo los puntos disponibles en la ventana, incluso en este modo.

Por defecto elimina puntos no finitos, distancias menores de 0,2 m y tags de baja confianza MID-360 (valor 2 en cualquiera de los tres campos de dos bits). `--min-range-m` ajusta la distancia y `--keep-low-confidence` conserva esos tags; los puntos no finitos se rechazan siempre. No elimina etiquetas reservadas ni confianza media. La inspección informa rechazos sólo en la ventana solicitada, y totales de la captura completa por separado.

No sobrescribe carpetas existentes. Una salida sólo se considera completa cuando existe `metadata.json`; si falla la escritura se elimina únicamente la carpeta nueva que creó este proceso. Lee el archivo completo antes de exportar y de nuevo al escribir, con memoria acotada por un registro más el índice por segundos. El PLY/CSV de varios segundos puede ocupar decenas de MB; es material de calibración, no un formato final de mapa.

Pruebas sintéticas, sin hardware:

```sh
clang++ -std=c++20 tests/calibration_export/make_fixture.cpp \
  -Icore/storage/include -Icore/common/include -Icore/imu/include -Icore/timesync/include \
  build/mac-release/core/libatlas_core.a -o build/calibration_export_fixture
python3 tests/calibration_export/test_export.py \
  build/mac-release/apps/atlas_calibration_export build/calibration_export_fixture
```
