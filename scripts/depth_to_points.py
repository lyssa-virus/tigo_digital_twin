#!/usr/bin/env python3
"""Processa a imagem de profundidade da câmera (simulada ou real).

Publica:
  /camera/points  (PointCloud2)  nuvem de pontos 3D, no frame da imagem (camera_optical_link)
  /camera/scan    (LaserScan)    "lidar virtual" feito com a câmera: menor distância por coluna,
                                 numa faixa de linhas ao redor do centro da imagem. Frame camera_link.

O /camera/scan serve para comparar o que a câmera enxerga com o /scan do lidar.
Aceita profundidade 32FC1 (metros, como no Gazebo) e 16UC1 (milímetros, comum em câmeras reais).
Rode com: --ros-args -p use_sim_time:=true   (na simulação)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField


def image_to_depth(msg):
    """Converte sensor_msgs/Image de profundidade em matriz float32 (metros)."""
    if msg.encoding == '32FC1':
        a = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
        return a[:, :msg.width]
    if msg.encoding == '16UC1':
        a = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
        return a[:, :msg.width].astype(np.float32) / 1000.0
    raise ValueError(f'Codificação de profundidade não suportada: {msg.encoding}')


def backproject(depth, fx, fy, cx, cy, stride, zmin, zmax):
    """Pontos 3D (N x 3) no frame óptico (x direita, y baixo, z à frente)."""
    d = depth[::stride, ::stride]
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    valid = np.isfinite(d) & (d > zmin) & (d < zmax)
    z = d[valid]
    x = (u[valid] - cx) * z / fx
    y = (v[valid] - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def depth_to_scan(depth, fx, cx, cy, band_rows, zmin, zmax):
    """Scan 2D a partir de uma faixa de linhas ao redor de cy.

    Retorna (ranges, angle_min, angle_increment). Ângulo positivo = esquerda (convenção do LaserScan).
    """
    h, w = depth.shape
    r0 = int(max(0, round(cy) - band_rows))
    r1 = int(min(h, round(cy) + band_rows + 1))
    band = depth[r0:r1, :]
    u = np.arange(w, dtype=np.float32)
    x = (u - cx) / fx                                   # x/z de cada coluna
    z = np.where(np.isfinite(band) & (band > zmin) & (band < zmax), band, np.inf)
    rng = z * np.sqrt(1.0 + x * x)[None, :]             # distância horizontal = z * sqrt(1 + (x/z)^2)
    col = rng.min(axis=0)                               # menor distância de cada coluna
    ranges = col[::-1]                                  # coluna w-1 (direita) tem o menor ângulo
    ang_min = -math.atan((w - 1 - cx) / fx)
    ang_max = math.atan(cx / fx)
    inc = (ang_max - ang_min) / (w - 1)
    return ranges, ang_min, inc


class DepthToPoints(Node):
    def __init__(self):
        super().__init__('depth_to_points')
        base = '/ascamera_hp60c/camera_publisher/depth0'
        self.declare_parameter('depth_topic', f'{base}/image_raw')
        self.declare_parameter('info_topic', f'{base}/camera_info')
        self.declare_parameter('points_topic', '/camera/points')
        self.declare_parameter('scan_topic', '/camera/scan')
        self.declare_parameter('scan_frame', 'camera_link')
        self.declare_parameter('stride', 4)          # usa 1 pixel a cada 4 (menos pontos, menos CPU)
        self.declare_parameter('every', 3)           # processa 1 imagem a cada 3 (30 Hz -> 10 Hz)
        self.declare_parameter('zmin', 0.10)
        self.declare_parameter('zmax', 10.0)
        self.declare_parameter('band_rows', 8)       # linhas acima/abaixo do centro usadas no scan

        g = self.get_parameter
        self.stride = g('stride').value
        self.every = g('every').value
        self.zmin = g('zmin').value
        self.zmax = g('zmax').value
        self.band_rows = g('band_rows').value
        self.scan_frame = g('scan_frame').value
        self.K = None
        self.count = 0

        self.pub_points = self.create_publisher(PointCloud2, g('points_topic').value, 5)
        self.pub_scan = self.create_publisher(LaserScan, g('scan_topic').value, 5)
        self.create_subscription(CameraInfo, g('info_topic').value, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, g('depth_topic').value, self.on_depth, qos_profile_sensor_data)
        self.get_logger().info('Aguardando camera_info e imagem de profundidade...')

    def on_info(self, msg):
        if self.K is None:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])   # fx, fy, cx, cy
            self.get_logger().info('camera_info recebido: fx=%.1f fy=%.1f cx=%.1f cy=%.1f' % self.K)

    def on_depth(self, msg):
        if self.K is None:
            return
        self.count += 1
        if self.count % self.every:
            return
        fx, fy, cx, cy = self.K
        try:
            depth = image_to_depth(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        pts = backproject(depth, fx, fy, cx, cy, self.stride, self.zmin, self.zmax)
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = pts.shape[0]
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud.width
        cloud.is_dense = True
        cloud.data = pts.tobytes()
        self.pub_points.publish(cloud)

        ranges, ang_min, inc = depth_to_scan(depth, fx, cx, cy, self.band_rows, self.zmin, self.zmax)
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.scan_frame
        scan.angle_min = float(ang_min)
        scan.angle_max = float(ang_min + inc * (len(ranges) - 1))
        scan.angle_increment = float(inc)
        scan.range_min = float(self.zmin)
        scan.range_max = float(self.zmax)
        scan.ranges = ranges.astype(np.float32).tolist()
        self.pub_scan.publish(scan)


def main():
    rclpy.init()
    rclpy.spin(DepthToPoints())


if __name__ == '__main__':
    main()
