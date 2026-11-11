#!/usr/bin/env python3
"""Converte /odom (nav_msgs/Odometry) em TF odom -> base_link.

Igual na simulação e no robô real: o que muda é só quem publica /odom
(sim_odometry.py na simulação, tigo_base no robô). Se o tigo_base já publicar
esse TF, NÃO rode este nó.
Parâmetro: odom_topic (padrão /odom); use_sim_time (padrão true; false no robô real).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/odom')
        topic = self.get_parameter('odom_topic').value
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.cb, 20)
        self.get_logger().info(f"Convertendo {topic} em TF odom->base_link")

    def cb(self, msg):
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = msg.header.frame_id or 'odom'
        t.child_frame_id = msg.child_frame_id or 'base_link'
        p = msg.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomToTF())


if __name__ == '__main__':
    main()
