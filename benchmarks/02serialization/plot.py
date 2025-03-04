import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Load the Excel file
file_path = './logs/benchmarks/serialization.xlsx'
data = pd.read_excel(file_path)

plt.rcParams.update({'font.size': 17})
# Display the first few rows of the dataframe to understand its structure
data.head()

protocols = data['Protocol']
throughput_serialize = data['Serialize Throughput (MB/s)']
throughput_deserialize = data['Deserialize Throughput (MB/s)']
x = np.arange(len(protocols))  # the label locations
width = 0.35  # the width of the bars

# Adjusting the width of the bars to make them thinner
width = 0.25  # New width for thinner bars

# Plotting with adjusted bar width
fig = plt.figure(figsize=(10, 8))

# Throughput Plot for Serialization and Deserialization with thinner bars
ax1 = fig.add_subplot(111)
rects1 = ax1.bar(x - width/2, throughput_serialize, width,
                 label='Serialize', color='skyblue')
rects2 = ax1.bar(x + width/2, throughput_deserialize, width,
                 label='Deserialize', color='lightgreen')

# Adding labels, title, and custom x-axis tick labels, etc.
ax1.set_ylabel('Throughput (MB/s)')
ax1.set_xticks(x)
ax1.set_xticklabels(protocols)
ax1.legend(loc='upper left')
ax1.bar_label(rects1, padding=3, fmt='%.2f')
ax1.bar_label(rects2, padding=3, fmt='%.2f')
ax1.set_yscale("log")
ax1.grid(True, which="both", ls="--", linewidth=0.5)

plt.tight_layout()
plt.savefig("./logs/benchmarks/serialization.png")
