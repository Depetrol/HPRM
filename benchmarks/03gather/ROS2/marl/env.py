import time
import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
import numpy as np
from sensor_msgs.msg import Image
import pickle


class Env(Node):
    def __init__(self):
        super().__init__('env')

        # Subscribers
        self.policy1 = Subscriber(self, Image, 'policy_topic1')
        self.policy2 = Subscriber(self, Image, 'policy_topic2')
        self.policy3 = Subscriber(self, Image, 'policy_topic3')
        self.policy4 = Subscriber(self, Image, 'policy_topic4')

        # Synchronize the subscribers
        ats = TimeSynchronizer(
            [self.policy1, self.policy2, self.policy3, self.policy4], 10)
        ats.registerCallback(self.callback)

        # Publisher
        self.publisher = self.create_publisher(
            Image, 'env_topic', 10)

        # Simulation
        self.round_num = 0
        self.start_time = None
        self.prev_time = None

        # Create the NumPy array
        n_rows = 62500  # 62500 rows is 1MB
        val = np.random.random((n_rows, 2))  # 2 columns for x and y
        self.send_message(val)

    def callback(self, p1, p2, p3, p4):
        d1 = pickle.loads(p1.data)
        d2 = pickle.loads(p2.data)
        d3 = pickle.loads(p3.data)
        d4 = pickle.loads(p4.data)

        # first round
        if int(self.round_num) == 0:
            self.start_time = time.time()
            self.prev_time = self.start_time

        # print round number
        print("Episode: "+str(self.round_num))
        self.round_num += 1

        # print Time Taken
        cur_time = time.time()
        print(f"Time taken: {cur_time - self.start_time:.5f} seconds")
        print(f"Overhead: {cur_time - self.prev_time - 0.5:.5f} seconds\n")
        self.prev_time = cur_time

        self.send_message(d1)

    def send_message(self, val):
        msg = Image()
        msg.data = pickle.dumps(val)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    env_node = Env()
    rclpy.spin(env_node)
    env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
