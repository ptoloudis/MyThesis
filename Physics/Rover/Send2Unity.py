import time
from pymavlink import mavutil
import socket
import json

# Set the frequency of messages to be sent to Unity
frequency = 1 / 120

# Connect to the ArduPilot
connection = mavutil.mavlink_connection('udp:192.168.0.8:14551')

# Wait for the heartbeat message to find the system ID
connection.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Create a socket connection to Unity
unity_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
unity_address = ('127.0.0.1', 5005)

# Request position and attitude data
connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
x, y, z = 0, 0, 0
roll, pitch, yaw = 0, 0, 0

while True:
    start_time = time.time()

    msg = connection.recv_match(blocking=True)
    if msg.get_type() == 'LOCAL_POSITION_NED':
        x = msg.x
        y = msg.y
        z = msg.z
    elif msg.get_type() == 'ATTITUDE':
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        
    # Send data to Unity
    data = {
        'position': [x, y, z],
        'attitude': [roll, pitch, yaw]
    }
#    print(data)
    unity_socket.sendto(json.dumps(data).encode(), unity_address)

    # Sleep to maintain 60 Hz frequency
    elapsed_time = time.time() - start_time
    sleep_time = max(0, frequency - elapsed_time)
    time.sleep(sleep_time)
