#!/usr/bin/env python3
"""
generate_warehouse_map.py
Genera warehouse_map.png — mapa 2D binario del almacén definido en warehouse.sdf.

Coordenadas del mundo (metros):
  Almacén: 10 x 8 m, centrado en (0, 0)
  x: [-5, +5]   y: [-4, +4]

Paso C de la tarea — dimensiones del grid:
  Resolución: 0.05 m / pixel
  Tamaño imagen: 200 x 160 px  (10/0.05 x 8/0.05)
  Origen (esquina inferior izquierda en coords. del mundo): (-5.0, -4.0)

Convención de imagen:
  - Blanco (255) = espacio libre (robot puede estar aquí)
  - Negro   (0)  = obstáculo (pared, pallet, pilar)
  - El eje Y del mundo apunta hacia arriba, pero en imagen Y apunta hacia abajo,
    así que invertimos al dibujar (row = (MAP_H - 1) - world_to_row(y))
"""

import numpy as np
import cv2

# ── Parámetros del mapa (Paso C) ──────────────────────────────────────────────
RES      = 0.05          # metros por pixel
ORIGIN_X = -5.0          # x del mundo en pixel (0,0) de la imagen
ORIGIN_Y = -4.0          # y del mundo en pixel (0,0) de la imagen
W_M, H_M = 10.0, 8.0    # tamaño del mundo en metros
MAP_W    = int(W_M / RES)   # 200 px
MAP_H    = int(H_M / RES)   # 160 px

# ── Funciones de conversión ────────────────────────────────────────────────────
def w2px(x):
    """Coordenada X del mundo → columna pixel."""
    return int((x - ORIGIN_X) / RES)

def w2py(y):
    """Coordenada Y del mundo → fila pixel (invertida)."""
    row = int((y - ORIGIN_Y) / RES)
    return (MAP_H - 1) - row   # invertir eje Y

def draw_box(img, cx, cy, sx, sy, color=0, margin=0.0):
    """Dibuja un rectángulo centrado en (cx,cy) con tamaño (sx,sy) en metros."""
    x0 = w2px(cx - sx/2 - margin)
    x1 = w2px(cx + sx/2 + margin)
    y0 = w2py(cy + sy/2 + margin)   # y0 < y1 en imagen (arriba = menor row)
    y1 = w2py(cy - sy/2 - margin)
    x0, x1 = sorted([x0, x1])
    y0, y1 = sorted([y0, y1])
    x0 = max(0, x0); y0 = max(0, y0)
    x1 = min(MAP_W-1, x1); y1 = min(MAP_H-1, y1)
    img[y0:y1+1, x0:x1+1] = color

# ── Generar mapa ───────────────────────────────────────────────────────────────
img = np.ones((MAP_H, MAP_W), dtype=np.uint8) * 255   # todo libre (blanco)

# Paredes exteriores (grosor 0.15 m)
T = 0.15
# Norte
draw_box(img,  0.0,  4.075, 10.15, T)
# Sur
draw_box(img,  0.0, -4.075, 10.15, T)
# Este
draw_box(img,  5.075, 0.0,  T, 8.15)
# Oeste
draw_box(img, -5.075, 0.0,  T, 8.15)

# Pallets Fila A (x = -3.5)
draw_box(img, -3.5, -2.5, 0.8, 0.6)
draw_box(img, -3.5,  0.0, 0.8, 0.6)
draw_box(img, -3.5,  2.5, 0.8, 0.6)

# Pallets Fila B (x = 0.0)
draw_box(img,  0.0, -3.0, 0.7, 0.6)
draw_box(img,  0.0,  3.0, 0.7, 0.6)

# Pallets Fila C (x = +3.5)
draw_box(img,  3.5, -2.5, 0.8, 0.6)
draw_box(img,  3.5,  0.0, 0.8, 0.6)
draw_box(img,  3.5,  2.5, 0.8, 0.6)

# Pilares (0.3 x 0.3 m)
draw_box(img, -4.2,  3.2, 0.3, 0.3)
draw_box(img,  4.2, -3.2, 0.3, 0.3)

# ── Guardar ───────────────────────────────────────────────────────────────────
OUTPUT = 'warehouse_map.png'
cv2.imwrite(OUTPUT, img)

print(f'Mapa generado: {OUTPUT}')
print(f'  Tamaño   : {MAP_W} x {MAP_H} px')
print(f'  Resolución: {RES} m/px  →  {W_M} x {H_M} m')
print(f'  Origen   : ({ORIGIN_X}, {ORIGIN_Y}) m')
print(f'  Libre    : {np.sum(img==255)} px  |  Obstáculo: {np.sum(img==0)} px')

# Mostrar (opcional)
try:
    big = cv2.resize(img, (MAP_W*4, MAP_H*4), interpolation=cv2.INTER_NEAREST)
    cv2.imshow('warehouse_map (x4)', big)
    print('Presiona cualquier tecla para cerrar la ventana...')
    cv2.waitKey(0)
    cv2.destroyAllWindows()
except Exception:
    pass
