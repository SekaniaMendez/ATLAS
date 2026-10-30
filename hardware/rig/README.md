# ATLAS: rig ajustable de bastón, iPhone y MID-360

## Kit STL para impresión

Descarga: [ATLAS_rig_STL.zip](ATLAS_rig_STL.zip), que contiene únicamente STL en
milímetros. Las cantidades de impresión aparecen en los nombres de archivo.

La variante física está en [stl/](stl/INSTRUCCIONES_IMPRESION.md): cinco diseños
para nueve piezas del rig, sin bastón ni equipos, más casquillos opcionales de
30 y 40 mm. Tiene uniones sólidas, pasos para tornillos y acceso a los cuatro M3.
Su cuna se separa del MID-360 para liberar el M12; el cambio de posición y la
calibración propuesta se documentan en esa carpeta. La maqueta original que se
describe a continuación conserva su configuración nominal.

Abrir **ATLAS_iPhone14Pro_MID360_Rig.blend**. Es un estudio de montaje editable;
las piezas están separadas por colecciones y la escena usa milímetros.
**El MID-360 y el teléfono quedan en un lateral del bastón**, que continúa hacia
arriba para el GNSS; el soporte no ocupa su punta. El eje del bastón está a
115 mm en +Y del LiDAR por defecto. Mover ese eje no altera la calibración entre
LiDAR y teléfono. El disco naranja superior solo reserva un volumen de 80 × 20 mm
para GNSS: no representa un modelo ni su rosca, todavía no especificados.
Configuración inicial: iPhone 14 Pro (147.5 × 71.5 × 7.85 mm), bastón de 35 mm,
abrazaderas partidas con casquillos para el intervalo de 30–40 mm. Cada diámetro
requiere su casquillo; no se propone apretar una pieza rígida de 40 mm hasta 30 mm.

## Ajustar en Blender

1. Abrir el área **Scripting / Text Editor**, seleccionar el texto incorporado
   **ATLAS_RIG.py** y ejecutar **Run Script** (Alt+P con el cursor en ese editor).
   El archivo no exige habilitar ejecución automática al abrir.
2. En la vista 3D, pulsar **N → ATLAS**. Elegir el teléfono en el menú; están los
   perfiles 14 Pro/Pro Max, 15 Pro/Pro Max y 16 Pro/Pro Max del proyecto.
3. Cambiar diámetro, separación lateral del bastón, altura libre para GNSS, espacio para funda, centro de lente, desplazamiento o
   rotación; pulsar **Actualizar rig desde parámetros**. Las medidas del teléfono
   se cargan del catálogo al cambiar de modelo. `phone_case_extra_mm` es por lado.
4. **Exportar calibración propuesta** guarda `calibration-proposed.json` junto al
   `.blend`. Siempre lleva `verified: false`. Revisar reproyección con el equipo
   real antes de copiar sus valores al catálogo de la app.

Los parámetros completos también están en las propiedades personalizadas del
objeto **ATLAS_PARAMETERS**, en el texto incorporado `rig_config.json` y en el
archivo externo homónimo. La regeneración reemplaza los objetos de las siete
colecciones del rig; colocar modificaciones manuales en otra colección antes de
regenerar. Guardar el `.blend` conserva las propiedades actuales. Al ejecutar el
generador desde terminal se vuelve a partir del JSON externo.

## Medidas y marcos

Se importa el catálogo real de `apps/ios/ATLASViewer/ATLASViewer/Resources/iphone-mid360-calibrations.json`.
El perfil inicial se conserva: R por filas `[0,0,-1; 0,-1,0; -1,0,0]`,
t `[75,-25,-31]` mm en `p_cámara = R p_lidar + t`.
Por tanto el centro de cámara en coordenadas LiDAR es **[-31,-25,75] mm**;
no es el vector t interpretado directamente como posición del teléfono.
`LIDAR_FRAME` y `CAMERA_OPTICAL_FRAME` muestran estos ejes; cámara mira hacia -Z.
`PHONE_BODY_FRAME` orienta la carcasa en vertical y permite localizar el lente.

Las cotas de carcasa vienen del catálogo existente. Este no incluye la posición
medida del centro de cada lente respecto al borde físico del teléfono. Los valores
iniciales **12 mm desde izquierda, 12 mm desde arriba y 2 mm de saliente** son una
maqueta editable, **no cotas oficiales ni una calibración medida**. El módulo de
cámaras se representa de forma esquemática. Medir con la funda real y la lente 1×.
Cambiar esos tres valores mueve la carcasa alrededor del centro óptico; no cambia
por sí solo la calibración LiDAR→cámara. Los controles `phone_shift_*` y rotaciones
sí cambian el centro/ejes ópticos y la calibración exportada.

El [manual oficial Livox v1.2](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)
(páginas impresas 12 y 19) da envolvente 65 × 65 × 60 mm, origen LiDAR 47 mm sobre
la base, montaje de cuatro M3 con patrón 48 × 36 mm y rosca de 5 mm de profundidad.
La placa tiene pasos de 3.4 mm. El cuerpo óptico y el conector se simplifican para
visualización; el STEP original está disponible en las
[descargas de Livox](https://www.livoxtech.com/mid-360/downloads).

## Lo que falta cerrar físicamente

`fit-report.json` conserva la comprobación geométrica inicial. **El perfil nominal
sitúa el teléfono en el espacio del conector M12 trasero** con la envolvente
modelada; además el teléfono sobresale sobre el plano óptico y puede tapar un
sector del LiDAR. Se conserva esa posición para hacer visible el problema del
perfil actual. Usar los desplazamientos para definir la posición real y luego
exportar la calibración: no ocultar el problema cambiando solo el dibujo.

El casquillo de 35 mm, mordazas, placa ranurada, montante y pomos están modelados.
Hay que comprobar la ruta de cable, acceso a pantalla/botones, recorrido real de
las correderas, uniones entre piezas, rigidez y longitud de tornillos antes de
fabricar. El kit STL separado no tiene una validación física de cargas. La exportación de calibración es una propuesta, no una
prueba de alineación física ni temporal de los sensores.

## Regenerar

```sh
/Applications/Blender.app/Contents/MacOS/Blender --background \
  --python hardware/rig/build_rig.py -- \
  --output "$PWD/hardware/rig" --render
```

`rig-preview.png` es una vista del conjunto. Los dos JSON y el script están
incorporados al `.blend`, de modo que el panel funciona aun si se mueve el archivo.
