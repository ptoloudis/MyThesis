import os
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import numpy as np

# Folder where your 9999 files are located
folder_path = 'JSON/diff'
keywords = ("Vel", "Pos", "Att", "Gyro", "Accel")
data = {key: [] for key in ("Vel", "Pos", "Att", "Gyro", "Accel")}
mode = int(input("Mode(1=Read, 2=Plot, 3= All): "))
name = input("File name: ")

# Function to read the values from a single file
def read_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

        current_key = None
        for line in lines:
            clean_line = line.rstrip('\n')  # Remove trailing newline
            if any(keyword in clean_line for keyword in keywords):
                current_key = clean_line
            else:
                normalized_string = clean_line.replace(',', '.')
                values = normalized_string.split()
                if current_key:  # Ensure current_key is set
                    data[current_key].append({
                        'x': float(values[0]),
                        'y': float(values[1]),
                        'z': float(values[2])
                    })

            
def numerical_sort(files):
    # Extract numbers from filenames and sort based on these numbers
    def extract_number(filename):
        try:
            # Extract number before the extension
            return int(filename.split('.')[0])
        except ValueError:
            return float('inf')  # If no number found, place it at the end

    return sorted(files, key=extract_number)

if mode == 1 or mode == 3:
# Get a list of all files
    files = [f for f in os.listdir(folder_path) if f.endswith('.txt')]

    # Sort files numerically
    sorted_files = numerical_sort(files)

    # Read each file in the sorted order
    for file_name in sorted_files:
        file_path = os.path.join(folder_path, file_name)
        # print(f"Reading file: {file_path}")
        read_file(file_path)

    # Prepare data for saving
    combined_data = []
    for key, values in data.items():
        for entry in values:
            combined_data.append([key, entry['x'], entry['y'], entry['z']])

    # Convert to numpy array
    combined_data = np.array(combined_data, dtype=object)

    # Save the combined data in one .npy file
    np.save('JSON/' + name + '_data.npy', combined_data)

    print("All data saved in one .npy file")

if mode == 2 or mode == 3:
# Load all data
    data = np.load('JSON/' + name + '_data.npy', allow_pickle=True)

    # Plotting
    # Create subplots for X, Y, and Z values combined
    current_key = None
    _data = {}  # Ensure data is a dictionary, not a NumPy array

    for i in range(data.shape[0]):  # Assuming data is your NumPy data
        if any(keyword in data[i][0] for keyword in keywords):
            current_key = data[i][0]  # Use the string as the dictionary key
            
            # Initialize data[current_key] as a list if it doesn't exist
            if current_key not in _data:
                _data[current_key] = []
            
            _data[current_key].append({
                'x': float(data[i][1]),
                'y': float(data[i][2]),
                'z': float(data[i][3])
            })
    # Plotting
    # Create a figure and a set of subplots
    fig, axes = plt.subplots(nrows=len(_data), ncols=1, figsize=(10, 6))

    # If there's only one subplot, axes will not be a list, so make it iterable
    if len(_data) == 1:
        axes = [axes]

    # Plot X values in one window
    fig1, axes1 = plt.subplots(len(_data), 1, figsize=(10, 6))
    fig1.canvas.setWindowTitle(name + ' X Data')
    for ax, (key, values) in zip(axes1, _data.items()):
        x_values = [entry['x'] for entry in values]
        ax.plot(x_values, label='X', color='blue')
        ax.set_title(f'{key} X Data')
        ax.set_xlabel('Index')
        ax.set_ylabel('Value')
        # Format y-axis to prevent scientific notation
        ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=False))
        ax.ticklabel_format(style='plain', axis='y')
        ax.legend()

    # Plot Y values in a second window
    fig2, axes2 = plt.subplots(len(_data), 1, figsize=(10, 6))
    fig2.canvas.setWindowTitle(name + ' Y Data')
    for ax, (key, values) in zip(axes2, _data.items()):
        y_values = [entry['y'] for entry in values]
        ax.plot(y_values, label='Y', color='green')
        ax.set_title(f'{key} Y Data')
        ax.set_xlabel('Index')
        ax.set_ylabel('Value')
        # Format y-axis to prevent scientific notation
        ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=False))
        ax.ticklabel_format(style='plain', axis='y')
        ax.legend()

    # Plot Z values in a third window
    fig3, axes3 = plt.subplots(len(_data), 1, figsize=(10, 6))
    fig3.canvas.setWindowTitle(name + ' Z Data')
    for ax, (key, values) in zip(axes3, _data.items()):
        z_values = [entry['z'] for entry in values]
        ax.plot(z_values, label='Z', color='red')
        ax.set_title(f'{key} Z Data')
        ax.set_xlabel('Index')
        ax.set_ylabel('Value')
        # Format y-axis to prevent scientific notation
        ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=False))
        ax.ticklabel_format(style='plain', axis='y')
        ax.legend()


    plt.tight_layout()
    plt.show()

    print(f"Done")