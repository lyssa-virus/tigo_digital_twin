#!/usr/bin/env python3
"""Publica o TF odom -> base_link a partir da pose real do TIGO no Gazebo.

O plugin de rodas mecanum não publica odometria nesta versão do Gazebo, então
usamos /world/lacea_world/pose/info (pose "verdadeira" de todos os objetos).
Isso equivale a uma odometria perfeita, sem deriva: serve para testar SLAM em
simulação, mas NÃO para avaliar o desempenho da odometria do robô real.

O frame 'odom' é a pose inicial do robô (o robô começa em odom = 0,0,0).
Parâmetro: child_name (padrão 'tigo'), o nome do modelo no Gazebo.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def relative_pose(x0, y0, yaw0, x, y, yaw):
    """Pose (x, y, yaw) expressa no referencial inicial (x0, y0, yaw0)."""
    dx, dy = x - x0, y - y0
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    return c * dx - s * dy, s * dx + c * dy, math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('child_name', 'tigo')
        self.child_name = self.get_parameter('child_name').value
        self.br = TransformBroadcaster(self)
        self.initial = None
        self.warned = False
        self.create_subscription(TFMessage, '/world/lacea_world/pose/info', self.cb, 10)
        self.get_logger().info(f"Aguardando a pose de '{self.child_name}' em /world/lacea_world/pose/info")

    def cb(self, msg):
        match = [t for t in msg.transforms if t.child_frame_id == self.child_name]
        if not match:
            if not self.warned:
                nomes = sorted({t.child_frame_id for t in msg.transforms})
                self.get_logger().warn(
                    f"Pose '{self.child_name}' não encontrada. Frames recebidos: {nomes}. "
                    "Rode com: --ros-args -p child_name:=NOME")
                self.warned = True
            return
        t = match[0]
        x, y = t.transform.translation.x, t.transform.translation.y
        yaw = quat_to_yaw(t.transform.rotation)
        if self.initial is None:
            self.initial = (x, y, yaw)
            self.get_logger().info(f"Pose inicial: x={x:.2f} y={y:.2f} yaw={yaw:.2f}. TF odom->base_link ativo.")
        rx, ry, ryaw = relative_pose(*self.initial, x, y, yaw)

        out = TransformStamped()
        # A pose do Gazebo pode vir sem carimbo de tempo; usa o relógio da simulação (/clock)
        stamp = t.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.transform.translation.x = rx
        out.transform.translation.y = ry
        out.transform.translation.z = 0.0
        out.transform.rotation.z = math.sin(ryaw / 2.0)
        out.transform.rotation.w = math.cos(ryaw / 2.0)
        self.br.sendTransform(out)


def main():
    rclpy.init()
    rclpy.spin(OdomToTF())


if __name__ == '__main__':
    main()
