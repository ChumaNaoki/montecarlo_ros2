# SPDX-FileCopyrightText: 2025 Chuma Naoki
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

rclpy.init()
node = Node("result")
n = 0

def cb(msg):
    global node
    global n
    n += 1
    node.get_logger().info(f"試行回数:{n} 円周率: {msg.data}")

def main():
    node.create_subscription(Float32, "random_pi_estimator", cb, 10)
    rclpy.spin(node)
