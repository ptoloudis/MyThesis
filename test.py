import os
import json
import matplotlib.pyplot as plt

timestamps = []
gyro_x = []
gyro_y = []
gyro_z = []
accel_x = []
accel_y = []
accel_z = []
position_x = []
position_y = []
position_z = []
quaternion_x = []
quaternion_y = []
quaternion_z = []
quaternion_w = []
velocity_x = []
velocity_y = []
velocity_z = []
N = []
D = []

# Function to load JSON data from a file
def load_json(filename):
    with open(filename, 'r') as f:
        # Read the first line containing the series of numbers
        numbers_line = f.readline().strip()
        # Read the second line containing the JSON-like data
        json_line = f.readline()
        
        # Process the series of numbers
        numbers = list(map(int, numbers_line.split()))
        if numbers[0] <= 1100:
            return
        
        # Process the JSON-like data
        N.append(numbers)
        data = json.loads(json_line)
        get_data(data)

        return

# Function to extract data and plot
def get_data(json_data):

    timestamps.append(json_data["timestamp"])
    gyro = json_data["imu"]["gyro"]
    accel = json_data["imu"]["accel_body"]
    position = json_data["position"]
    quaternion = json_data["quaternion"]
    velocity = json_data["velocity"]

    gyro_x.append(gyro[0])
    gyro_y.append(gyro[1])
    gyro_z.append(gyro[2])

    accel_x.append(accel[0])
    accel_y.append(accel[1])
    accel_z.append(accel[2])

    position_x.append(position[0])
    position_y.append(position[1])
    position_z.append(position[2])

    quaternion_x.append(quaternion[0])
    quaternion_y.append(quaternion[1])
    quaternion_z.append(quaternion[2])
    quaternion_w.append(quaternion[3])

    velocity_x.append(velocity[0])
    velocity_y.append(velocity[1])
    velocity_z.append(velocity[2])

def plot_data():
    # Convert the data to columns for plotting
    data = list(zip(*N))  # Transpose rows to columns

    # Generate x values (indices)
    x = range(len(data[0]))

    # Create a figure and axis for subplots
    fig1, axs = plt.subplots(5, 1, figsize=(10, 8))

    for i in x:
        diff = max(N[i]) - min(N[i])
        D.append(diff)
    
    axs[0].plot(timestamps, D, 'o', label=f'Motor diff')
    axs[0].set_xlabel('Timestamps')
    axs[0].set_ylabel(f'Motor diff')
    axs[0].legend()

    # Plot each column in a separate subplot
    for i in range(4):
        axs[i+1].plot(timestamps, data[i], 'o', label=f'Motor {i}')
        axs[i+1].set_xlabel('Timestamps')
        axs[i+1].set_ylabel(f'Motor {i}')
        axs[i+1].legend()

    # Add a title to the figure
    fig1.suptitle('nx4 List Plot')

    # Adjust layout to prevent overlap
    fig1.tight_layout(rect=[0, 0, 1, 0.96])

    # Show the plot
    # plt.show()


    plt.figure(figsize=(14, 10))

    plt.subplot(3, 3, 1)
    plt.plot(timestamps, gyro_x, 'o', label='gyro_x')
    plt.title('Gyroscope Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()

    plt.subplot(3, 3, 2)
    plt.plot(timestamps, gyro_y, 'o', label='gyro_y')
    plt.title('Gyroscope Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()

    plt.subplot(3, 3, 3)
    plt.plot(timestamps, gyro_z, 'o', label='gyro_z')
    plt.title('Gyroscope Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()

    plt.subplot(3, 3, 4)
    plt.plot(timestamps, accel_x, 'o', label='accel_x')
    plt.title('Accelerometer Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()

    plt.subplot(3, 3, 5)
    plt.plot(timestamps, accel_y, 'o', label='accel_y')
    plt.title('Accelerometer Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()

    plt.subplot(3, 3, 6)
    plt.plot(timestamps, accel_z, 'o', label='accel_z')
    plt.title('Accelerometer Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend()


    plt.subplot(3, 3, 7)
    plt.plot(timestamps, position_x, 'o', label='position_x')
    plt.title('Position Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Position (m)')
    plt.legend()

    plt.subplot(3, 3, 8)
    plt.plot(timestamps, position_y, 'o', label='position_y')
    plt.title('Position Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Position (m)')
    plt.legend()

    plt.subplot(3, 3, 9)
    plt.plot(timestamps, position_z, 'o', label='position_z')
    plt.title('Position Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Position (m)')
    plt.legend()

    plt.tight_layout()
    plt.show()

    plt.subplot(3, 3, 1)
    plt.plot(timestamps, velocity_x, 'o', label='velocity_x')
    plt.title('Velocity Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.subplot(3, 3, 2)
    plt.plot(timestamps, velocity_y, 'o', label='velocity_y')
    plt.title('Velocity Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.subplot(3, 3, 3)
    plt.plot(timestamps, velocity_z, 'o', label='velocity_z')
    plt.title('Velocity Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.subplot(3, 3, 4)
    plt.plot(timestamps, quaternion_x, 'o', label='quaternion_x')
    plt.title('Quaternion Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Quaternion Value')
    plt.legend()

    plt.subplot(3, 3, 5)
    plt.plot(timestamps, quaternion_y, 'o', label='quaternion_y')
    plt.title('Quaternion Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Quaternion Value')
    plt.legend()

    plt.subplot(3, 3, 6)
    plt.plot(timestamps, quaternion_z, 'o', label='quaternion_z')
    plt.title('Quaternion Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Quaternion Value')
    plt.legend()

    plt.subplot(3, 3, 7)
    plt.plot(timestamps, quaternion_w, 'o', label='quaternion_w')
    plt.title('Quaternion Data')
    plt.xlabel('Timestamp')
    plt.ylabel('Quaternion Value')
    plt.legend()

    plt.tight_layout()
    plt.show()

# Path to the folder containing JSON files
folder_path = '~/JSON/unity'

# List all files in the directory
filenames = sorted(os.listdir(folder_path))

# Filter only .txt files (assuming your data files are .txt)
filenames = [f for f in filenames if f.endswith('.json')]

# Iterate over files in the folder
for filename in filenames:
    # print(filename)
    load_json(os.path.join(folder_path, filename))

print(f"Plotting data")
plot_data()

