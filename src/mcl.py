#!/usr/bin/env python3
"""
mcl.py — Monte Carlo Localization para Puzzlebot en almacén
Autor: Valeria Acerda

Pasos implementados:
  D. Muestreo de partículas (uniforme en espacio libre del mapa)
  E. Puntaje por partícula (likelihood field con LiDAR)
  F. Filtrado (resampling proporcional al peso)
  G. Dead reckoning (modelo cinemático diferencial desde odometría)
  H. Mover partículas según odometría
  I. Iterar continuamente a 10 Hz

Mapa: warehouse_map.png
  Resolución : 0.05 m/px
  Tamaño     : 200 x 160 px = 10 x 8 m
  Origen     : (-5.0, -4.0) m
"""

import math
import numpy as np
import cv2
import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header

# ─── Parámetros del mapa (Paso C) ────────────────────────────────────────────
MAP_PATH   = '/home/valeriaacerda/montecarlo-localization/maps/warehouse_map.png'
MAP_RES    = 0.05     # metros por pixel
MAP_ORIG_X = -5.0    # origen x del mapa en coords del mundo
MAP_ORIG_Y = -4.0    # origen y del mapa en coords del mundo

# ─── Parámetros del filtro ────────────────────────────────────────────────────
N_PARTICLES = 2000   # total de partículas (Paso D)
N_KEEP      = 1900    # mejores partículas que se conservan (Paso F)
SIGMA_HIT   = 0.30   # desviación gaussiana del sensor — más alto = menos agresivo
Z_MAX       = 6.0    # distancia máxima de rayo a considerar
N_RAYS      = 36     # rayos del LiDAR usados por iteración

# ─── Ruido del modelo de movimiento (Pasos G, H) ─────────────────────────────
SIGMA_XY    = 0.05   # m — ruido traslacional (más alto = partículas más dispersas)
SIGMA_THETA = 0.05   # rad — ruido rotacional


# ─── Utilidades ──────────────────────────────────────────────────────────────
def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ─── Nodo MCL ────────────────────────────────────────────────────────────────
class MCL(Node):
    def __init__(self):
        super().__init__('mcl')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])

        # ── B. Cargar mapa conocido ──────────────────────────────────────────
        raw = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)
        if raw is None:
            raise FileNotFoundError(f'No se encontró el mapa: {MAP_PATH}')
        self.map_h, self.map_w = raw.shape
        self.map_free = (raw > 200).astype(np.uint8)  # 1=libre 0=obstáculo

        # ── E. Pre-calcular likelihood field ────────────────────────────────
        obstacle = (self.map_free == 0).astype(np.uint8)
        dist_px  = cv2.distanceTransform(1 - obstacle, cv2.DIST_L2, 5)
        self.dist_field = dist_px * MAP_RES  # en metros

        self.get_logger().info(
            f'Mapa cargado: {self.map_w}x{self.map_h} px | '
            f'{self.map_w*MAP_RES:.1f}x{self.map_h*MAP_RES:.1f} m')

        # ── D. Muestreo inicial uniforme de partículas ───────────────────────
        self.particles = self._sample_uniform(N_PARTICLES)
        self.weights   = np.ones(N_PARTICLES) / N_PARTICLES

        # Estado interno
        self.last_odom = None
        self.iter = 0
        self.iter = 0
        self.delta     = (0.0, 0.0)
        self.scan      = None

        # ── Suscriptores ─────────────────────────────────────────────────────
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(
            Odometry, '/model/puzzlebot/odometry', self.cb_odom, 10)
        self.create_subscription(
            LaserScan, '/scan', self.cb_scan, qos_sensor)

        # ── Publicadores ─────────────────────────────────────────────────────
        self.pub_particles = self.create_publisher(PoseArray,   '/mcl/particles', 10)
        self.pub_pose      = self.create_publisher(PoseStamped, '/mcl/pose',      10)

        # ── Loop principal 10 Hz (Paso I) ────────────────────────────────────
        self.create_timer(0.1, self.step)
        self.get_logger().info(f'MCL iniciado — {N_PARTICLES} partículas')

    # ── D. Muestreo uniforme en espacio libre ────────────────────────────────
    def _sample_uniform(self, n):
        free_rows, free_cols = np.where(self.map_free == 1)
        idx    = np.random.choice(len(free_rows), size=n, replace=True)
        xs     = free_cols[idx] * MAP_RES + MAP_ORIG_X
        ys     = (self.map_h - 1 - free_rows[idx]) * MAP_RES + MAP_ORIG_Y
        thetas = np.random.uniform(-math.pi, math.pi, size=n)
        return np.column_stack([xs, ys, thetas]).astype(np.float64)

    # ── G. Dead reckoning — calcula Δd y Δθ ─────────────────────────────────
    def cb_odom(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.last_odom is None:
            self.last_odom = (x, y, yaw)
            return

        dx     = x - self.last_odom[0]
        dy     = y - self.last_odom[1]
        dtheta = math.atan2(
            math.sin(yaw - self.last_odom[2]),
            math.cos(yaw - self.last_odom[2]))
        dd = math.hypot(dx, dy)
        if math.cos(self.last_odom[2])*dx + math.sin(self.last_odom[2])*dy < 0:
            dd = -dd

        self.delta     = (dd, dtheta)
        self.last_odom = (x, y, yaw)

    def cb_scan(self, msg):
        self.scan = msg

    # ── Ciclo principal ──────────────────────────────────────────────────────
    def step(self):
        if self.scan is None or self.last_odom is None:
            return

        # ── H. Mover partículas (predicción) ────────────────────────────────
        dd, dtheta = self.delta
        N = len(self.particles)

        # Siempre añade ruido para mantener diversidad
        noise_d = np.random.normal(0, SIGMA_XY,    N)
        noise_t = np.random.normal(0, SIGMA_THETA, N)

        self.particles[:, 2] += dtheta + noise_t
        self.particles[:, 2]  = np.arctan2(
            np.sin(self.particles[:, 2]),
            np.cos(self.particles[:, 2]))

        step_d = dd + noise_d
        self.particles[:, 0] += step_d * np.cos(self.particles[:, 2])
        self.particles[:, 1] += step_d * np.sin(self.particles[:, 2])

        # ── E. Puntaje (likelihood field) ───────────────────────────────────
        self.weights = self._score(self.particles, self.scan)

        total = self.weights.sum()
        if total < 1e-12:
            self.get_logger().warn('Pesos colapsaron — re-inicializando')
            self.particles = self._sample_uniform(N_PARTICLES)
            self.weights   = np.ones(N_PARTICLES) / N_PARTICLES
        else:
            self.weights /= total

        # ── F. Filtrar y re-muestrear ────────────────────────────────────────
        self._resample()

        # Publica
        self._publish()

    # ── E. Score vectorizado ─────────────────────────────────────────────────
    def _score(self, particles, scan):
        ranges  = np.array(scan.ranges, dtype=np.float32)
        n_total = len(ranges)
        step    = max(1, n_total // N_RAYS)
        idxs    = np.arange(0, n_total, step)
        angles  = scan.angle_min + idxs * scan.angle_increment
        r       = ranges[idxs]

        valid  = np.isfinite(r) & (r > scan.range_min) & (r < Z_MAX)
        angles = angles[valid]
        r      = r[valid]
        if len(r) == 0:
            return np.ones(len(particles))

        px  = particles[:, 0:1]
        py  = particles[:, 1:2]
        pth = particles[:, 2:3]

        ex = px + r[None, :] * np.cos(pth + angles[None, :])
        ey = py + r[None, :] * np.sin(pth + angles[None, :])

        cols = ((ex - MAP_ORIG_X) / MAP_RES).astype(np.int32)
        rows = (self.map_h - 1 - (ey - MAP_ORIG_Y) / MAP_RES).astype(np.int32)

        in_bounds = (
            (cols >= 0) & (cols < self.map_w) &
            (rows >= 0) & (rows < self.map_h))

        cols_c = np.clip(cols, 0, self.map_w - 1)
        rows_c = np.clip(rows, 0, self.map_h - 1)

        d = np.where(in_bounds, self.dist_field[rows_c, cols_c], Z_MAX)

        log_w = -0.5 * np.sum((d / SIGMA_HIT) ** 2, axis=1)
        log_w -= log_w.max()
        return np.exp(log_w)

    # ── F. Low-variance resampling ───────────────────────────────────────────
    def _resample(self):
        N = len(self.particles)

        # Selecciona top N_KEEP
        top_idx = np.argpartition(-self.weights, N_KEEP)[:N_KEEP]
        top_p   = self.particles[top_idx]
        top_w   = self.weights[top_idx]
        top_w  /= top_w.sum()

        # Low-variance resampling
        positions  = (np.arange(N) + np.random.uniform()) / N
        cumulative = np.cumsum(top_w)
        indices    = np.zeros(N, dtype=np.int32)
        i = j = 0
        while i < N and j < N_KEEP:
            if positions[i] < cumulative[j]:
                indices[i] = j
                i += 1
            else:
                j += 1
        if i < N:
            indices[i:] = N_KEEP - 1

        self.particles = top_p[indices].copy()

        # Jitter mayor para mantener diversidad visual
        self.particles[:, 0] += np.random.normal(0, 0.5, N)
        self.particles[:, 1] += np.random.normal(0, 0.5, N)
        self.particles[:, 2] += np.random.normal(0, 0.03, N)
        self.weights = np.ones(N) / N

    # ── Publicar ─────────────────────────────────────────────────────────────
    def _publish(self):
        now = self.get_clock().now().to_msg()

        # Nube de partículas — frame map
        pa = PoseArray()
        pa.header = Header(stamp=now, frame_id='map')
        for x, y, th in self.particles[:, :3]:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation = quaternion_from_yaw(float(th))
            pa.poses.append(p)
        self.pub_particles.publish(pa)

        # Estimación — media ponderada
        mx  = float(self.particles[:, 0].mean())
        my  = float(self.particles[:, 1].mean())
        mth = float(math.atan2(
            np.sin(self.particles[:, 2]).mean(),
            np.cos(self.particles[:, 2]).mean()))

        ps = PoseStamped()
        ps.header = Header(stamp=now, frame_id='map')
        ps.pose.position.x = mx
        ps.pose.position.y = my
        ps.pose.orientation = quaternion_from_yaw(mth)
        self.pub_pose.publish(ps)

        self.get_logger().info(
            f'Pose estimada: x={mx:.2f} y={my:.2f} th={math.degrees(mth):.1f}°',
            throttle_duration_sec=2.0)


# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = MCL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()