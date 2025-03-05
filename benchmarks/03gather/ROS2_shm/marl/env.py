import time
import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
import numpy as np
from sensor_msgs.msg import Image
import pickle
from multiprocessing import shared_memory
import os

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
        self.shm = None

        # Create the NumPy array
        self.object_size = 50
        self.n_rows = 62500 * self.object_size  # 62500 rows is 1MB
        val = np.random.random((self.n_rows, 2))  # 2 columns for x and y
        self.prev_time = self.start_time = time.time()
        self.send_message(val)

    def callback(self, p1, p2, p3, p4):
        d1 = self.deserialization(p1.data)
        d2 = self.deserialization(p2.data)
        d3 = self.deserialization(p3.data)
        d4 = self.deserialization(p4.data)

        cur_time = time.time()
        # print round number
        self.round_num += 1
        print("Episode: "+str(self.round_num))

        # print Time Taken
        print(f"Time taken: {cur_time - self.start_time:.5f} seconds")
        print(f"Overhead: {cur_time - self.prev_time - 0.5:.5f} seconds\n")
        if self.round_num == 10:
            if not os.path.exists("../../../logs/benchmarks"):
                os.makedirs("../../../logs/benchmarks")
            if os.path.exists("../../../logs/benchmarks/gather.pkl"):    
                with open("../../../logs/benchmarks/gather.pkl", "rb") as f:
                    results = pickle.load(f)
            else:
                results = {}
            if "ros2_shm" not in results.keys():
                results["ros2_shm"] = {} 
            results["ros2_shm"][str(self.object_size)] = (cur_time - self.start_time) / self.round_num - 0.5,
            with open("../../../logs/benchmarks/gather.pkl", "wb") as f:
                pickle.dump(results, f)
            print("Benchmarking done.")
            return
        self.prev_time = cur_time

        self.send_message(d1)

    def serialization(self, val):
        if self.shm:
            self.shm.close()
            self.shm.unlink()

        serialized_array = pickle.dumps(val)
        self.size = len(serialized_array)
        # Create and Write shared memory
        self.shm = shared_memory.SharedMemory(create=True, size=self.size)
        self.shm.buf[:self.size] = serialized_array
        return pickle.dumps((self.shm.name, self.size, self.n_rows))

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
