import pickle
import os
import matplotlib.pyplot as plt

# Load the pickle file
pickle_file_path = "./logs/benchmarks/delay.pkl"

with open(pickle_file_path, "rb") as f:
    results = pickle.load(f)

# Sort object sizes for plotting
object_sizes = sorted(results.keys())

# Extract times
plasma_times = [results[size]["plasma"] for size in object_sizes]
shared_times = [results[size]["shared_memory"] for size in object_sizes]

# Compute throughput (MB/s) = Object size (MB) / time (seconds)
plasma_throughputs = [
    size / results[size]["plasma"] if results[size]["plasma"] != 0 else 0 
    for size in object_sizes
]
shared_throughputs = [
    size / results[size]["shared_memory"] if results[size]["shared_memory"] != 0 else 0
    for size in object_sizes
]

# Create figure
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# Plot Time vs. Object Size
ax1.plot(object_sizes, plasma_times, marker='o', label='Plasma')
ax1.plot(object_sizes, shared_times, marker='x', label='Shared Memory')

ax1.set_title("Time Taken by Plasma and Shared Memory")
ax1.set_xlabel("Object Size (MB)")
ax1.set_ylabel("Time (seconds)")
ax1.legend()

# Plot Throughput vs. Object Size
ax2.plot(object_sizes, plasma_throughputs, marker='o', label='Plasma Throughput')
ax2.plot(object_sizes, shared_throughputs, marker='x', label='Shared Memory Throughput')

ax2.set_title("Throughput of Plasma and Shared Memory")
ax2.set_xlabel("Object Size (MB)")
ax2.set_ylabel("Throughput (MB/s)")
ax2.legend()

plt.tight_layout()

plt.savefig("./logs/benchmarks/delay.png")
plt.close(fig)