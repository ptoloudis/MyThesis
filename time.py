import numpy as np

# Read data from file
file_path = "timeR.txt"

with open(file_path, 'r') as file:
    data = file.readlines()

# Parsing the data
m_values = []
f_values = []

for line in data:
    if line.startswith('M:'):
        m_values.append(float(line[2:]))
    elif line.startswith('F:'):
        f_values.append(float(line[2:]))

# Convert lists to numpy arrays for easier calculations
m_values = np.array(m_values)
f_values = np.array(f_values)

# Calculate statistics
m_stats = {
    'mean': np.mean(m_values),
    'median': np.median(m_values),
    'std': np.std(m_values),
    'min': np.min(m_values),
    'max': np.max(m_values)
}

f_stats = {
    'mean': np.mean(f_values),
    'median': np.median(f_values),
    'std': np.std(f_values),
    'min': np.min(f_values),
    'max': np.max(f_values)
}

# Print the results
print("Statistics for M values:")
for stat, value in m_stats.items():
    print(f"{stat.capitalize()}: {value:.10f}")

print("\nStatistics for F values:")
for stat, value in f_stats.items():
    print(f"{stat.capitalize()}: {value:.10f}")
