import time
import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
import numpy as np
from sensor_msgs.msg import Image
import pickle
from multiprocessing import shared_memory


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
        self.shm = None
        self.size = 50000164

        # Create the NumPy array
        self.n_rows = 62500*50  # 62500 rows is 1MB, 5MB, 10MB, 25MB
        val = np.random.random((self.n_rows, 2))  # 2 columns for x and y
        self.send_message(val)

    def callback(self, p1, p2, p3, p4):
        d1 = self.deserialization(p1.data)
        d2 = self.deserialization(p2.data)
        d3 = self.deserialization(p3.data)
        d4 = self.deserialization(p4.data)

        # first round
        if int(self.round_num) == 0:
            self.start_time = time.time()
            self.prev_time = self.start_time

        # print round number
        print("Episode: "+str(self.round_num))
        self.round_num += 1

        # print Time Taken
        cur_time = time.time()
        print(f"Time taken: {cur_time - self.start_time:.4f} seconds")
        print(f"Overhead: {cur_time - self.prev_time - 0.5:.4f} seconds\n")
        self.prev_time = cur_time

        self.send_message(d1)

    def serialization(self, val):
        if self.shm:
            self.shm.close()
            self.shm.unlink()

        serialized_array = pickle.dumps(val)
        print(len(serialized_array))
        # Create and Write shared memory
        self.shm = shared_memory.SharedMemory(create=True, size=self.size)
        self.shm.buf[:self.size] = serialized_array
        return pickle.dumps(self.shm.name)

    def deserialization(self, memory_name):
        local_shm = shared_memory.SharedMemory(name=pickle.loads(memory_name))
        data = pickle.loads(local_shm.buf[:self.size])
        n_rows = self.n_rows
        array = np.frombuffer(data, dtype=np.float64).reshape(n_rows, 2)
        return array

    def send_message(self, val):
        msg = Image()
        msg.data = self.serialization(val)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    env_node = Env()
    rclpy.spin(env_node)
    env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
