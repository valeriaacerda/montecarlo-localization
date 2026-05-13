#!/usr/bin/env python3
"""
demo_mcl.py — Demostración visual del algoritmo Monte Carlo Localization
Autor: Valeria Acerda

Muestra:
  1. El mapa del almacén (warehouse_map.png)
  2. Las partículas dispersas inicialmente (Paso D)
  3. Cómo se puntúan (Paso E)
  4. Cómo convergen tras filtrado (Paso F)
  5. Cómo se mueven con dead reckoning (Pasos G, H)
"""

import numpy as np
import cv2
import math

# ── Parámetros (mismo que mcl.py) ─────────────────────────────────────────────
MAP_PATH   = '/home/valeriaacerda/montecarlo-localization/maps/warehouse_map.png'
MAP_RES    = 0.05
MAP_ORIG_X = -5.0
MAP_ORIG_Y = -4.0
N_PARTICLES = 500

# ── Cargar mapa ───────────────────────────────────────────────────────────────
raw = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)
map_h, map_w = raw.shape
map_free = (raw > 200).astype(np.uint8)

# Escala para visualización (x4)
SCALE = 4
vis_w = map_w * SCALE
vis_h = map_h * SCALE

def world_to_vis(x, y):
    """Convierte coords del mundo a pixeles de visualización."""
    col = int((x - MAP_ORIG_X) / MAP_RES) * SCALE
    row = int((map_h - 1 - (y - MAP_ORIG_Y) / MAP_RES)) * SCALE
    return col, row

def sample_particles(n):
    """Paso D: muestreo uniforme en espacio libre."""
    free_rows, free_cols = np.where(map_free == 1)
    idx = np.random.choice(len(free_rows), size=n, replace=True)
    xs = free_cols[idx] * MAP_RES + MAP_ORIG_X
    ys = (map_h - 1 - free_rows[idx]) * MAP_RES + MAP_ORIG_Y
    thetas = np.random.uniform(-math.pi, math.pi, n)
    return np.column_stack([xs, ys, thetas])

def draw_frame(particles, weights=None, robot_pos=None, title=""):
    """Dibuja el mapa con partículas encima."""
    # Mapa en color (blanco=libre, gris oscuro=obstáculo)
    frame = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
    frame = cv2.resize(frame, (vis_w, vis_h), interpolation=cv2.INTER_NEAREST)

    # Colorea el mapa: libre=blanco, obstáculo=gris oscuro
    obstacle_mask = cv2.resize((map_free == 0).astype(np.uint8) * 255,
                               (vis_w, vis_h), interpolation=cv2.INTER_NEAREST)
    frame[obstacle_mask > 0] = [60, 60, 60]

    # Dibuja partículas
    for i, (x, y, th) in enumerate(particles):
        col, row = world_to_vis(x, y)
        if 0 <= col < vis_w and 0 <= row < vis_h:
            # Color según peso: azul (bajo peso) → rojo (alto peso)
            if weights is not None:
                w = float(weights[i])
                w_norm = min(1.0, w * len(weights))
                color = (int(255*(1-w_norm)), 50, int(255*w_norm))
            else:
                color = (255, 100, 0)  # azul por default

            # Dibuja flecha de orientación
            arrow_len = 8
            ex = int(col + arrow_len * math.cos(th))
            ey = int(row - arrow_len * math.sin(th))
            cv2.arrowedLine(frame, (col, row), (ex, ey), color, 1,
                           tipLength=0.4)

    # Dibuja robot real (si se conoce)
    if robot_pos is not None:
        rx, ry, rth = robot_pos
        rc, rr = world_to_vis(rx, ry)
        cv2.circle(frame, (rc, rr), 8, (0, 255, 0), -1)
        ex = int(rc + 15 * math.cos(rth))
        ey = int(rr - 15 * math.sin(rth))
        cv2.arrowedLine(frame, (rc, rr), (ex, ey), (0, 200, 0), 2)
        cv2.putText(frame, "Robot real", (rc+10, rr-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    # Estimación MCL (media de partículas)
    mx = float(particles[:, 0].mean())
    my = float(particles[:, 1].mean())
    mth = math.atan2(np.sin(particles[:, 2]).mean(),
                     np.cos(particles[:, 2]).mean())
    mc, mr = world_to_vis(mx, my)
    cv2.circle(frame, (mc, mr), 6, (0, 0, 255), -1)
    ex = int(mc + 15 * math.cos(mth))
    ey = int(mr - 15 * math.sin(mth))
    cv2.arrowedLine(frame, (mc, mr), (ex, ey), (0, 0, 255), 2)
    cv2.putText(frame, "Estimacion MCL", (mc+10, mr+15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    # Leyenda
    cv2.putText(frame, title, (10, 20),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.putText(frame, f"Particulas: {len(particles)}", (10, 45),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.putText(frame, f"Estimacion: x={mx:.2f} y={my:.2f}", (10, 65),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Leyenda de colores
    cv2.putText(frame, "Azul=baja prob  Rojo=alta prob", (10, vis_h-40),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    cv2.putText(frame, "Verde=robot real  Rojo=estimacion MCL", (10, vis_h-20),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    return frame


# ── Demo paso a paso ──────────────────────────────────────────────────────────
print("=" * 60)
print("DEMO: Monte Carlo Localization — Almacén Puzzlebot")
print("=" * 60)
print()
print("PASO B: Mapa del entorno conocido")
print(f"  Archivo : {MAP_PATH}")
print(f"  Tamaño  : {map_w} x {map_h} px")
print(f"  Resolución (Paso C): {MAP_RES} m/px")
print(f"  Mundo   : {map_w*MAP_RES:.1f} x {map_h*MAP_RES:.1f} m")
print(f"  Origen  : ({MAP_ORIG_X}, {MAP_ORIG_Y}) m")
print(f"  Libre   : {(map_free==1).sum()} px")
print(f"  Obstáculo: {(map_free==0).sum()} px")
print()

# Posición real del robot (donde spawneó en Gazebo)
robot_real = (-4.0, 0.0, 0.0)

print("PASO D: Muestreo inicial de partículas")
particles = sample_particles(N_PARTICLES)
print(f"  {N_PARTICLES} partículas distribuidas uniformemente en espacio libre")
print(f"  x: [{particles[:,0].min():.1f}, {particles[:,0].max():.1f}]")
print(f"  y: [{particles[:,1].min():.1f}, {particles[:,1].max():.1f}]")
print()

frame_D = draw_frame(particles, robot_pos=robot_real,
                     title="Paso D: Muestreo uniforme inicial")
cv2.imshow("MCL Demo", frame_D)
print(">> Presiona cualquier tecla para continuar...")
cv2.waitKey(0)

print("PASO E: Puntaje por partícula (likelihood field)")
print("  Cada partícula recibe un peso P(z|x) basado en")
print("  la distancia de los rayos del LiDAR al obstáculo más cercano")
print("  Regla de Bayes: P(x|z) ∝ P(z|x) · P(x)")
print()

# Simula pesos (sin LiDAR real — muestra el concepto)
# Partículas cerca del robot real tienen mayor peso
dists = np.sqrt((particles[:,0] - robot_real[0])**2 +
                (particles[:,1] - robot_real[1])**2)
weights = np.exp(-0.5 * (dists / 1.5)**2)
weights /= weights.sum()

frame_E = draw_frame(particles, weights=weights, robot_pos=robot_real,
                     title="Paso E: Puntaje (azul=bajo, rojo=alto peso)")
cv2.imshow("MCL Demo", frame_E)
print(">> Presiona cualquier tecla para continuar...")
cv2.waitKey(0)

print("PASO F: Filtrado — conservar mejores partículas")
N_KEEP = 100
top_idx = np.argpartition(-weights, N_KEEP)[:N_KEEP]
top_p = particles[top_idx]
top_w = weights[top_idx]
top_w /= top_w.sum()

# Resample
indices = np.random.choice(N_KEEP, size=N_PARTICLES, p=top_w, replace=True)
particles = top_p[indices].copy()
particles[:, 0] += np.random.normal(0, 0.3, N_PARTICLES)
particles[:, 1] += np.random.normal(0, 0.3, N_PARTICLES)
particles[:, 2] += np.random.normal(0, 0.1, N_PARTICLES)
print(f"  Conservando top {N_KEEP}, re-muestreando {N_PARTICLES} partículas")
print()

frame_F = draw_frame(particles, robot_pos=robot_real,
                     title="Paso F: Filtrado — partículas convergen")
cv2.imshow("MCL Demo", frame_F)
print(">> Presiona cualquier tecla para continuar...")
cv2.waitKey(0)

print("PASO G + H: Dead reckoning — mover partículas")
print("  El robot avanza Δd=0.5m, Δθ=0.3rad")
print("  Modelo cinemático diferencial:")
print("    x_k = x_{k-1} + Δd·cos(θ_{k-1})")
print("    y_k = y_{k-1} + Δd·sin(θ_{k-1})")
print("    θ_k = θ_{k-1} + Δθ")
print()

dd, dtheta = 0.5, 0.3
particles[:, 2] += dtheta + np.random.normal(0, 0.05, N_PARTICLES)
particles[:, 0] += dd * np.cos(particles[:, 2]) + np.random.normal(0, 0.05, N_PARTICLES)
particles[:, 1] += dd * np.sin(particles[:, 2]) + np.random.normal(0, 0.05, N_PARTICLES)

robot_real = (robot_real[0] + dd*math.cos(robot_real[2]),
              robot_real[1] + dd*math.sin(robot_real[2]),
              robot_real[2] + dtheta)

frame_GH = draw_frame(particles, robot_pos=robot_real,
                      title="Pasos G+H: Movimiento con dead reckoning")
cv2.imshow("MCL Demo", frame_GH)
print(">> Presiona cualquier tecla para terminar...")
cv2.waitKey(0)

print("PASO I: Iterar desde Paso D")
print("  El ciclo se repite a 10 Hz en el nodo MCL de ROS 2")
print("  Con cada iteración las partículas convergen más")
print()
print("Demo completada.")
cv2.destroyAllWindows()