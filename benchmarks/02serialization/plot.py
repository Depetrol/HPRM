import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

file_path = './logs/benchmarks/serialization.xlsx'
data = pd.read_excel(file_path)

plt.rcParams.update({'font.size': 17})

protocols = data['Protocol']
throughput_serialize = data['Serialize Throughput (MB/s)']
throughput_deserialize = data['Deserialize Throughput (MB/s)']
x = np.arange(len(protocols))
width = 0.25
fig = plt.figure(figsize=(10, 8), dpi=300)

ax1 = fig.add_subplot(111)
rects1 = ax1.bar(x - width/2, throughput_serialize, width,
                 label='Serialize', color='skyblue')
rects2 = ax1.bar(x + width/2, throughput_deserialize, width,
                 label='Deserialize', color='lightgreen')

ax1.set_ylabel('Throughput (MB/s)', fontsize = 20)
ax1.set_xticks(x)
ax1.set_xticklabels(protocols)
ax1.legend(fontsize = 16)
ax1.bar_label(rects1, padding=3, fmt='%.2f')
ax1.bar_label(rects2, padding=3, fmt='%.2f')
ax1.set_yscale("log")
ax1.grid(True, which="both", ls="--", linewidth=0.5)
ax1.tick_params(axis='both', labelsize=20)

plt.tight_layout()
plt.savefig("./logs/benchmarks/serialization.png")
