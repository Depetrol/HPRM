import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# Load the updated data from an Excel file
result_df = pd.read_excel('plot.xlsx')  # Update this path as necessary
plt.rcParams.update({'font.size': 18})
# Clean the data
filtered_df = result_df.dropna().reset_index(drop=True)
filtered_df.columns = ['Configuration', '1MB', '5MB', '10MB', '25MB', '50MB']

# Data for plotting
object_sizes = ['1MB', '5MB', '10MB', '25MB', '50MB']
configurations = filtered_df['Configuration'].tolist()
data = filtered_df.iloc[:, 1:].values.T  # Transpose to align with object sizes

# Bar chart settings
bar_width = 0.15  # Adjusted for the number of configurations
index = np.arange(len(object_sizes))

# Set the style for the plot
sns.set_style("whitegrid")

# Colors for each configuration, extended to fit new configurations
colors = ['#6699ff', '#ff6666', '#9999ff', '#ff9933', '#66cc66']

# Plot each configuration
plt.figure(figsize=(12, 8))
for i, (config, color) in enumerate(zip(configurations, colors)):
    plt.bar(index + i * bar_width, data[:, i],
            bar_width, label=config, color=color)

plt.xlabel('Object Size')
plt.ylabel('Latency (milliseconds)')
#plt.title(
#    'Mean Latency of Broadcast and Gather on 4 nodes\nwith Different Object Sizes')
plt.xticks(index + bar_width * 2, object_sizes)
plt.yscale("log")

plt.legend()
plt.tight_layout()


# Save the updated plot to a file
plt.savefig("plot.png")  # Update this path as necessary
plt.show()
