import os
import pickle
import matplotlib.pyplot as plt
import numpy as np

if os.path.exists("./logs/benchmarks/gather.pkl"):    
    with open("./logs/benchmarks/gather.pkl", "rb") as f:
        results = pickle.load(f)

print(results)

data = results
# Prepare the data for plotting
object_sizes = ['1', '5', '10', '25', '50']
object_sizes_mb = [1, 5, 10, 25, 50]

latency_ros2 = [data['ros2'][size][0] * 1000 for size in object_sizes]  # Convert to milliseconds
latency_ros2_shm = [data['ros2_shm'][size][0] * 1000 for size in object_sizes]
latency_hprm_centralized = [data['hprm_centralized'][size][0] * 1000 for size in object_sizes]
latency_hprm_decentralized = [data['hprm_decentralized'][size][0] * 1000 for size in object_sizes]

latency_ros2 = [x * 1000 for x in latency_ros2]
latency_ros2_shm = [x * 1000 for x in latency_ros2_shm]
latency_hprm_centralized = [x * 1000 for x in latency_hprm_centralized]
latency_hprm_decentralized = [x * 1000 for x in latency_hprm_decentralized]

# Plotting the bar plot
bar_width = 0.2
index = np.arange(len(object_sizes_mb))

plt.figure(figsize=(8, 6))
plt.yscale('log')

plt.bar(index, latency_ros2, bar_width, label='ROS2 Humble')
plt.bar(index + bar_width, latency_ros2_shm, bar_width, label='ROS2 Humble (Shared Memory)')
plt.bar(index + 2 * bar_width, latency_hprm_centralized, bar_width, label='HPRM Centralized')
plt.bar(index + 3 * bar_width, latency_hprm_decentralized, bar_width, label='HPRM Decentralized')

plt.xlabel('Object Size (MB)')
plt.ylabel('Latency (milliseconds)')
plt.title('Latency vs Object Size')
plt.xticks(index + 1.5 * bar_width, object_sizes_mb)
plt.legend()
plt.grid(True, which="both", ls="--")

plt.savefig("./logs/benchmarks/gather.png")

