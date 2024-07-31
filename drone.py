from pymavlink import mavutil
import socket
import json

# Create a datagram socket
socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Once connected, use 'the_connection' to get and send messages
while 1:
    # Wait for a LOCAL_POSITION_NED message
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    
    # Extract local position data
    local_pos = str(msg.x) + "|" + str(msg.y) + "|" + str(msg.z)  # Local position in NED coordinates

    print(local_pos)

    socket.sendto(local_pos.encode('utf-8'),('', 5000))
