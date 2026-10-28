# SparseVoxelMap — lienzo 3D, fase 1

El mapa recibe puntos expresados en un marco local fijo, en metros. La identidad
de ese marco pertenece al proyecto: GNSS puede proporcionar una transformación
hacia ENU, pero el contenedor no interpreta latitud/longitud ni estima poses.
Los puntos de LiDAR deben transformarse antes de insertarlos. La resolución es
inmutable durante la vida del mapa; cambiarla requiere reconstruirlo.

## Organización y costo

La clave de voxel se calcula con `floor(coordenada / voxel_size_m)`. Cada bloque
agrupa `8 × 8 × 8` celdas. Una tabla hash localiza el bloque por sus coordenadas
enteras; un índice `x + 8*y + 64*z` localiza la celda en su array. La división
entera para bloques usa redondeo hacia menos infinito también en coordenadas
negativas. Las claves completas se comparan para resolver colisiones del hash.

- Búsqueda puntual: O(1) esperado; no es una garantía de latencia máxima.
- Inserción: O(1) esperado/amortizado, con asignaciones cuando crece el mapa.
- Vecindario cúbico de radio `r`: hasta `(2r+1)^3` búsquedas, incluyendo el centro.
- Consumir cambios: O(bloques modificados), sin recorrer el mapa completo.
- Diagnóstico de memoria: O(celdas asignadas), fuera del camino por punto.

Un bloque se asigna con su primera observación. Dentro del bloque se reserva
metadata para sus 512 celdas; las celdas sin observar no asignan muestras. Esto
favorece el acceso a vecinos, pero desperdicia metadata cuando las observaciones
están muy dispersas. No se afirma que use menos memoria que todos los diseños de
hash por voxel. El tamaño de bloque es una decisión inicial que debe contrastarse
con capturas reales.

Ausencia de voxel significa **desconocido**. La fase 1 no calcula espacio libre
mediante rayos ni genera superficies de colisión.

## Uso mínimo

```cpp
#include <atlas/slam/SparseVoxelMap.hpp>

using namespace atlas::slam;

SparseVoxelMap map({
    .voxel_size_m = 0.05,
    .bounds = MapBounds{-5, -4, 0, 5, 4, 4},
    .max_samples_per_voxel = 32,
});

MapPoint point{.x_m = 1.2, .y_m = -0.3, .z_m = 1.5,
               .rgb = {180, 170, 150}, .has_color = true};
const auto result = map.insert(point);
if (result == VoxelInsertResult::Inserted) {
  const auto key = *map.voxel_key_for(point);
  const auto address = SparseVoxelMap::address_for(key);
  const auto recovered_key = SparseVoxelMap::key_for_address(address);
  const auto center = map.voxel_center(key);
  const auto cell_box = map.voxel_bounds(key);
  const auto neighborhood = map.neighbors(key); // hasta 27 celdas observadas
  // recovered_key, center, cell_box y neighborhood: datos para consumidores.
}

map.set_bounds(MapBounds{-10, -8, -1, 10, 8, 6});
const auto dimensions = map.config().bounds->dimensions_m();
const auto memory = map.memory_stats();
const auto changes = map.take_changes();
```

## Contratos de la API

| Operación | Contrato |
| --- | --- |
| `set_bounds(box)` | Caja alineada a los ejes, mínimo incluido y máximo excluido. Se aplica a inserciones futuras; no recorta ni borra capturas. |
| `set_bounds(std::nullopt)` | Permite nuevas observaciones en toda la rejilla representable. |
| `voxel_key_for(point)` | Ignora límites de captura. Rechaza coordenadas no finitas o fuera del rango int32 de celdas. |
| `voxel_bounds(key)` / `voxel_center(key)` | Conversión a coordenadas métricas, sin asignar memoria. |
| `address_for(key)` / `key_for_address(address)` | Conversión reversible voxel↔bloque/índice. La inversa valida índice y rango. |
| `find` / `find_at` | Devuelve solo celdas observadas; sigue permitiendo leer capturas fuera de límites editados. |
| `find_block` / `blocks` | Acceso de solo lectura a bloques y celdas; el orden de la tabla hash no es estable. |
| `neighbors(key, radius)` | Celdas observadas en orden z/y/x; incluye el centro si existe. Radio permitido: 0–8. No es una consulta k-NN ni un radio métrico exacto. |
| `reserve_blocks(n)` | Reserva capacidad del índice hash; no crea bloques ni reserva todo el levantamiento. |
| `clear()` | Borra geometría y conteos de observaciones; conserva límites/resolución y emite reinicio. |

Los límites actuales son una caja de captura editable, no un polígono catastral.
Sus dimensiones describen el volumen de trabajo, no predicen la cantidad final de
superficies ni de memoria. No se asigna el volumen delimitado al configurar la caja.

La configuración rechaza resoluciones no positivas, subnormales o cuyo rango de
rejilla produciría coordenadas infinitas. La conversión usa la aritmética double
de los valores recibidos, sin tolerancias artificiales en los bordes. La resolución
espacial organiza los puntos: no garantiza precisión métrica del levantamiento.

## Muestras y futuras superficies

Cada voxel cuenta todas las observaciones aceptadas y retiene las últimas `N`
muestras en un buffer circular. Los valores iniciales no quedan congelados al
llenarlo. `samples` está en orden de almacenamiento, que deja de ser cronológico
después del primer reemplazo. Un límite de cero ya no habilita almacenamiento
ilimitado: se rechaza como configuración inválida.

Esto es una política de retención acotada, no fusión geométrica, ni muestreo
estadísticamente representativo, ni estimación de confianza. Muchos impactos no
equivalen a observaciones independientes. La selección robusta, el ajuste de planos
y la evidencia temporal se incorporarán en las fases siguientes. El RGB se almacena
si el llamador lo proporciona; esta fase no conecta ni calibra la cámara.

## Notificaciones para IA, visualización u otros consumidores

Cada inserción aceptada incrementa la revisión global y la del bloque afectado.
Una modificación de límites incrementa la revisión global y activa una señal de
metadata. Las inserciones rechazadas no cambian geometría ni revisiones.

`take_changes()` devuelve una sola entrada por bloque pendiente, con su revisión
más reciente, y consume la cola. Un nuevo cambio en ese bloque lo vuelve a encolar.
El orden es el de la primera modificación pendiente, no el orden espacial.
No hay umbral geométrico de relevancia en esta fase: cualquier observación aceptada
marca cambios; el planificador de IA podrá agruparlos o filtrarlos posteriormente.

La primera lectura y cada `clear()` emiten `reset=true`. El consumidor descarta su
caché anterior antes de procesar los bloques del lote. Si se insertan puntos entre
`clear()` y la lectura, el lote contiene solo los bloques nuevos. La revisión global
es monótona, incluso al limpiar; no se reutilizan números de revisión.

La cola tiene **un consumidor**. Para varios módulos, un coordinador consume y
distribuye el mismo lote. El lote contiene identificadores y revisiones, no una
copia de geometría. Todo acceso al mapa requiere sincronización externa; para IA
asíncrona, el coordinador debe copiar las regiones necesarias bajo esa sincronización.
Los punteros a voxels sobreviven inserciones/rehash, pero quedan invalidados por
limpieza, destrucción o reemplazo del mapa. El contenido de las muestras puede cambiar.

## Verificación reproducible

Sin sensores, ROS ni librerías nuevas:

```sh
cmake -S . -B build/voxel-test -G Ninja -DCMAKE_BUILD_TYPE=Release -DATLAS_BUILD_APPS=OFF -DATLAS_ENABLE_LIVOX=OFF -DATLAS_ENABLE_UI_STREAM=ON -DATLAS_BUILD_TESTS=ON -DATLAS_WARNINGS_AS_ERRORS=ON
cmake --build build/voxel-test
ctest --test-dir build/voxel-test --output-on-failure
build/voxel-test/tests/atlas_sparse_voxel_room_test
```

`SparseVoxelMapTest` verifica coordenadas negativas, extremos int32, conversiones,
límites, datos inválidos, reemplazo de muestras, cambios/reinicio y consultas contra
un modelo de referencia independiente. Sus comprobaciones se ejecutan en Release.

`SparseVoxelRoomTest` construye las seis superficies de una habitación sintética
de 10×8×4 m a 6,25 cm, mezcla el orden de entrada y consulta impactos y espacios
desconocidos. Comprueba que repetir capturas no aumenta las capacidades después de
llenar los buffers. Publica tiempos orientativos, sin umbrales de velocidad frágiles.

Una ejecución inicial en AppleClang 21, Release, dio:

- 76.424 voxels observados, 1.048 bloques y 536.576 celdas asignadas, frente a
  1.310.720 celdas posibles del volumen completo.
- 25,78 MiB estimados con cuatro muestras por voxel; 49,33 MiB para una rejilla
  completamente densa con el mismo tipo de celda y las mismas muestras.
- Aproximadamente 6,4 millones de inserciones iniciales/s y 19,7 millones de
  consultas/s. Son mediciones del contenedor en una escena sintética, no del SLAM.

`memory_stats()` contabiliza payload de bloques, capacidad de muestras, cola e
índice estimado. Excluye overhead del asignador/nodos y arrays del benchmark; no
mide RSS del proceso. El ahorro reportado se compara con almacenamiento denso,
no con la primera implementación hasheada por voxel.

## Alcance restante

Esta entrega completa el contenedor de la fase 1. No está conectado al flujo de
captura existente. El total de bloques aún puede crecer: presupuesto global de RAM,
descarga a disco, grabación/reproducción y persistencia corresponden a la fase 2.
La [fase 2](CAPTURE_AND_SYNCHRONIZATION.md) añade persistencia y una caché separada
`PagedVoxelMap`; `SparseVoxelMap` sigue siendo el contenedor en memoria sin expulsión.
La procedencia de capturas, submapas corregibles, superficies, SLAM y exportación
Unreal pertenecen a las siguientes capas. Los límites de captura y la retención
por voxel no sustituyen un presupuesto global de memoria.
