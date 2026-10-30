# STL del rig lateral ATLAS

Este kit contiene **solo las piezas del soporte**. No imprimir el bastón, teléfono,
MID-360, GNSS ni tornillos: no forman parte de los STL. La punta del bastón queda
libre. Es una variante física del estudio Blender, con uniones reales y pasos de
herramienta; el archivo de la maqueta original se conserva aparte.

**Unidades: milímetros; escala 100 %.** Los STL ya están en mm. No aplicar el
factor 0.001 de las unidades de visualización de Blender.

## Qué enviar al servicio de impresión

| STL | Copias | Uso |
| --- | ---: | --- |
| `01_cuerpo_lateral_x1.stl` | 1 | Placa LiDAR, brazos, montante y dos medias abrazaderas integradas |
| `02_tapa_abrazadera_x2.stl` | 2 | Cierre desmontable de las dos abrazaderas |
| `03_cuna_iphone14pro_x1.stl` | 1 | Cuna con apoyo inferior, lateral fijo y acceso al conector de carga |
| `04_mordaza_movil_x1.stl` | 1 | Lateral ajustable con dos tornillos y tuercas cautivas |
| `05_casquillo_35mm_x4.stl` | 4 | Dos pares de medios casquillos para bastón de 35 mm |

Son **9 piezas físicas, de 5 diseños distintos**, para el montaje inicial de
35 mm. En `opcionales/` hay medios casquillos de 30 y 40 mm: sustituyen a los de
35 mm, no se montan encima. Medir el diámetro real antes de imprimir los casquillos;
para diámetros intermedios hay que generar el diámetro correspondiente.

La cuna está dimensionada para **iPhone 14 Pro sin funda**. La mordaza permite
abrir el ancho, pero eso no aumenta la profundidad de la cuna; una funda gruesa
requiere ajustar el diseño. Se prevé una holgura de unos 0.4 mm respecto a la
carcasa, no una precisión garantizada de cualquier impresora.

El cuerpo mide aproximadamente **78.9 × 191.9 × 194.3 mm**. Comprobar el volumen
útil de la impresora, considerando también borde y soportes. Los STL no incluyen
soportes ni G-code. El cuerpo requiere estudiar soportes en el laminador,
especialmente bajo la placa y los brazos; mantener libres los taladros y pozos.
La orientación incluida de la cuna deja las cavidades de los tornillos hacia arriba.

Como punto de partida de prototipo: piezas rígidas en PETG/ASA y casquillos en TPU.
La densidad, perímetros, orientación y material deben definirlos quien imprime;
no se han ensayado resistencia, fatiga, temperatura ni vibración. Comprobar primero
montaje y sujeción sin equipos. No se entrega como pieza estructural certificada.

## Tornillos del MID-360

Sí: la placa incorpora **cuatro agujeros pasantes Ø3.4 mm para M3**, con centros
a **48 × 36 mm**, y **pozos inferiores Ø7.6 mm** para introducir las cabezas y
la llave aunque haya un brazo debajo. La placa contra el sensor conserva **6 mm**
de espesor. Se comprobaron las cuatro trayectorias de acceso por rayos en la malla.

El [manual oficial Livox, página impresa 19](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)
indica cuatro roscas M3 de **5 mm de profundidad**. Con placa de 6 mm, un M3×10
penetra nominalmente 4 mm sin arandela, menos si se añade una. No superar la
profundidad del sensor ni usar la longitud total del pozo como espesor de apoyo.

Tornillería metálica prevista (no imprimir):

| Elemento | Cantidad | Destino |
| --- | ---: | --- |
| M3×10, cabeza cilíndrica | 4 | MID-360 a placa |
| M4×30, cabeza cilíndrica | 4 | Abrazaderas del bastón |
| M4×20, cabeza cilíndrica | 2 | Cuna al montante |
| M4×10, cabeza cilíndrica | 2 | Mordaza móvil |
| Tuerca hexagonal M4 | 8 | Cuatro tapas, dos montante, dos mordaza |
| Arandela M4 | 4 | Bajo cabezas exteriores de las abrazaderas |

Los alojamientos hexagonales tienen **7.3 mm entre caras** y los pasos M4 son
Ø4.5 mm. Los rebajes de cabezas de la cuna son Ø8.2 mm, profundidad 4.3 mm;
verificar que las cabezas compradas encajan. Las longitudes son nominales y deben
comprobarse con el apilado real impreso; no imprimir roscas dentro del sensor.

## Montaje

1. Colocar las tuercas cautivas en las tapas, montante y mordaza.
2. Poner dos medios casquillos en cada abrazadera, presentar las tapas y los
   cuatro M4×30 con arandelas. Ajustar altura y orientación lateral antes de apretar.
3. Fijar la cuna al montante con los dos M4×20, desde los rebajes que quedan bajo
   la espalda del teléfono. Las cabezas deben quedar por debajo de esa superficie.
4. Colocar la mordaza móvil con sus dos M4×10 y deslizarla para sujetar el teléfono.
   Comprobar acceso a botones, cámara y carga; las esquinas del teléfono no deben
   soportar presión excesiva.
5. Atornillar el MID-360 con los cuatro M3 adecuados por debajo de la placa.

No es una exportación de los pomos decorativos de la maqueta: aquí las piezas se
sujetan mediante tornillos y tuercas metálicos reales.

## Posición del teléfono y calibración

La variante impresa desplaza el teléfono **−16 mm en X y +60 mm en Y** respecto al
perfil nominal, manteniendo la rotación. Esto deja margen para el conector M12
trasero. Se comprueba un volumen reservado de Ø20 × 50 mm para el enchufe/cable,
que es una reserva de diseño y no las medidas verificadas de tu cable.

`calibracion-variante-impresa.json` describe la posición prevista: traslación
LiDAR→cámara `[0.075, 0.035, -0.047]` m. Conserva `verified: false`: medir y comprobar
la reproyección después de montar. **No se cambió la calibración de la app.**
Las posiciones del centro óptico dentro del iPhone siguen siendo aproximadas;
las correcciones del montaje deben medirse. El soporte y el teléfono también
ocultan parte del campo del LiDAR; no se promete cobertura libre de 360°.

## Verificación digital

`validate_stl.py` verifica los 7 archivos: una componente cerrada por archivo,
orientación coherente, volumen positivo, sin triángulos degenerados y dimensiones
en mm. Eso no demuestra resistencia ni ajuste de la pieza realmente impresa.
También pasaron las 122 comprobaciones de `validate_print_fit.py` sobre el
ensamblaje: separación entre piezas y envolventes de equipos, casquillos para
30/35/40 mm y paso y soporte de los tornillos M4. El informe está en
`assembly-validation.json`. La reserva del M12 es geométrica; hay que comprobar
el enchufe y la curva del cable reales. Estas comprobaciones no sustituyen la
prueba física de montaje.
