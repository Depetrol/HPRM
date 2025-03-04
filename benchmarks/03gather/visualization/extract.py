import pandas as pd

# Load the excel file
file_name = 'lf_decentralized.xlsx'
df = pd.read_excel(file_name)

# Display the dataframe to understand its structure
df.head()
# Extract overhead times and convert them to integer arrays
overhead_times = []

# Loop through each column to find and process overhead times
for column in df.columns:
    # Extracting overhead time strings
    overhead_time_strings = df[column].dropna().apply(
        lambda x: x.split()[-2] if 'Overhead:' in x else None).dropna()
    # Converting to float and then to integer
    overhead_times_column = overhead_time_strings.apply(
        lambda x: int(float(x))).tolist()
    overhead_times.append(overhead_times_column)

# Correcting the approach to accurately extract overhead times as floats and then converting to integers

overhead_times_corrected = []

for column in df.columns:
    # Correctly extracting overhead time strings
    overhead_time_strings_corrected = df[column].dropna().apply(
        lambda x: x.split()[-2] if 'Overhead:' in x else None).dropna()
    # Correcting the conversion to float and then to integer
    overhead_times_column_corrected = overhead_time_strings_corrected.apply(lambda x: int(
        float(x)*1000)).tolist()  # Multiply by 1000 to capture milliseconds as integers
    overhead_times_corrected.append(overhead_times_column_corrected)

print(overhead_times_corrected)

# Column names from the data
columns = ["1MB", "5MB","10MB", "25MB", "50MB"]

# Calculating the average overhead times for each column and putting them in a dictionary
average_overhead_times = {}

for column, times in zip(columns, overhead_times_corrected):
    average_overhead = sum(times) / len(times)
    average_overhead_times[column] = average_overhead

print(average_overhead_times)

import numpy as np

std_deviation_overhead_times = {}

for column, times in zip(columns, overhead_times_corrected):
    std_deviation = np.std(times)
    std_deviation_overhead_times[column] = std_deviation

print(std_deviation_overhead_times)

print(file_name)