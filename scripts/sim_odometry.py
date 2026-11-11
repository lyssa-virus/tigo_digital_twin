#!/usr/bin/env python3
"""Odometria SIMULADA com erro realista. Publica /odom (nav_msgs/Odometry).

SÓ PARA A SIMULAÇÃO. No robô real, quem publica /odom é o tigo_base (encoders).
Assim, tudo que vem depois (odom_to_tf, SLAM, RViz) roda igual nos dois casos.

Como funciona: lê a pose verdadeira do TIGO em /world/lacea_world/pose/info,
calcula quanto o robô andou entre duas leituras (no referencial do robô) e
acrescenta erro antes de integrar, como as rodas mecanum fazem na prática:
  - erro sistemático de escala (raio de roda errado, deslizamento), sorteado
    uma vez por execução: sx, sy (x/y) e sw (giro);
  - ruído proporcional à distância/ângulo percorrido;
  - deriva constante no giro (yaw_bias, rad/s).
Parâmetros (--ros-args -p nome:=valor):
  noise_scale  multiplica todos os erros (0 = odometria perfeita, 1 = padrão, 2 = pior)
  seed         semente do sorteio (mesmo valor = mesmo erro, bom para comparar)
  scale_std_xy / scale_std_yaw / noise_trans / noise_rot / yaw_bias
  child_name   nome do modelo no Gazebo (padrão 'tigo')
"""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NoisyIntegrator:
    """Integra deslocamentos verdadeiros com erro. Sem ROS, fácil de testar."""

    def __init__(self, noise_scale=1.0, seed=1, scale_std_xy=0.02, scale_std_yaw=0.03,
                 noise_trans=0.02, noise_rot=0.02, yaw_bias=0.0):
        self.k = noise_scale
        self.rng = random.Random(seed)
        self.sx = 1.0 + self.k * self.rng.gauss(0, scale_std_xy)
        self.sy = 1.0 + self.k * self.rng.gauss(0, scale_std_xy * 2.0)  # lateral derrapa mais
        self.sw = 1.0 + self.k * self.rng.gauss(0, scale_std_yaw)
        self.noise_trans = noise_trans
        self.noise_rot = noise_rot
        self.yaw_bias = yaw_bias
        self.x = self.y = self.yaw = 0.0

    def step(self, dxb, dyb, dyaw, dt):
        """dxb, dyb, dyaw: deslocamento verdadeiro no referencial do robô."""
        k, g = self.k, self.rng.gauss
        d = math.hypot(dxb, dyb)
        nx = dxb * self.sx + k * g(0, self.noise_trans * d)
        ny = dyb * self.sy + k * g(0, self.noise_trans * d)
        nw = dyaw * self.sw + k * g(0, self.noise_rot * abs(dyaw)) + k * self.yaw_bias * dt
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += nx * c - ny * s
        self.y += nx * s + ny * c
        self.yaw = wrap(self.yaw + nw)
        return nx, ny, nw


class SimOdometry(Node):
    MIN_DT = 0.02  # s: ignora leituras muito próximas

    def __init__(self):
        super().__init__('sim_odometry', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        defaults = dict(child_name='tigo', noise_scale=1.0, seed=1, scale_std_xy=0.02,
                        scale_std_yaw=0.03, noise_trans=0.02, noise_rot=0.02, yaw_bias=0.0)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = {k: self.get_parameter(k).value for k in defaults}
        self.child_name = p.pop('child_name')
        self.integ = NoisyIntegrator(**p)
        self.get_logger().info(
            f"Erro de escala sorteado: x={self.integ.sx:.3f} y={self.integ.sy:.3f} giro={self.integ.sw:.3f} "
            f"(noise_scale={p['noise_scale']}, seed={p['seed']})")
        self.last = None  # (t, x, y, yaw) da pose verdadeira
        self.warned = False
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(TFMessage, '/world/lacea_world/pose/info', self.cb, 10)

    def cb(self, msg):
        match = [t for t in msg.transforms if t.child_frame_id == self.child_name]
        if not match:
            if not self.warned:
                nomes = sorted({t.child_frame_id for t in msg.transforms})
                self.get_logger().warn(f"Pose '{self.child_name}' não encontrada. Frames: {nomes}")
                self.warned = True
            return
        tr = match[0].transform
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        x, y, yaw = tr.translation.x, tr.translation.y, quat_to_yaw(tr.rotation)
        if self.last is None:
            self.last = (t, x, y, yaw)
            self.get_logger().info("Publicando /odom (odometria simulada com erro).")
            return
        t0, x0, y0, yaw0 = self.last
        dt = t - t0
        if dt < self.MIN_DT:
            return
        dxw, dyw = x - x0, y - y0
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        dxb, dyb = c * dxw - s * dyw, s * dxw + c * dyw
        dyaw = wrap(yaw - yaw0)
        nx, ny, nw = self.integ.step(dxb, dyb, dyaw, dt)
        self.last = (t, x, y, yaw)

        o = Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.integ.x
        o.pose.pose.position.y = self.integ.y
        o.pose.pose.orientation.z = math.sin(self.integ.yaw / 2.0)
        o.pose.pose.orientation.w = math.cos(self.integ.yaw / 2.0)
        o.twist.twist.linear.x = nx / dt
        o.twist.twist.linear.y = ny / dt
        o.twist.twist.angular.z = nw / dt
        for i, v in ((0, 0.05), (7, 0.05), (35, 0.1)):
            o.pose.covariance[i] = v
        for i, v in ((0, 0.02), (7, 0.02), (35, 0.05)):
            o.twist.covariance[i] = v
        self.pub.publish(o)


def main():
    rclpy.init()
    rclpy.spin(SimOdometry())


if __name__ == '__main__':
    main()
