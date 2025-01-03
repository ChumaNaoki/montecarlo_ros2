import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

rclpy.init()
node = Node("monte_carlo_publisher")

pub_random_pi_estimator = node.create_publisher(Float32, "random_pi_estimator", 10)

in_count = 0
all_count = 0

def pi_estimate():  # random_pi_estimator
    global in_count
    global all_count
    x = random.uniform(-1, 1)
    y = random.uniform(-1, 1)
    all_count += 1

    if x**2 + y**2 <= 1:
        in_count += 1

    pi = 4 * ( in_count / all_count )
    msg = Float32()
    msg.data = pi
    pub_random_pi_estimator.publish(msg)

def main():
    node.create_timer(0.3, pi_estimate)
    rclpy.spin(node)
