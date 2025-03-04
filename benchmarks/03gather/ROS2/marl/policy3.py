import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Preamble
import numpy as np
import time
import pickle


class Policy3(Node):
    def __init__(self):
        super().__init__('policy3')
        self.publisher_ = self.create_publisher(
            Image, 'policy_topic3', 10)
        self.subscription = self.create_subscription(
            Image,
            'env_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        received_np_array = pickle.loads(msg.data)
        time.sleep(0.5)
        self.send_message(received_np_array)

    def send_message(self, val):
        msg = Image()
        msg.data = pickle.dumps(val)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    policy_node = Policy3()
    rclpy.spin(policy_node)
    policy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
