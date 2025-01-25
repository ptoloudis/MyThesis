from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np
from pymavlink import mavutil

# Connect to the drone
vehicle = connect('192.168.0.8:14551', wait_ready=True)  # Replace with your connection string

#  Function to move the drone to a specific position from the current position
def goto_position_new(north, east, down):
    old_north, old_east, old_down = get_position()
    send_ned_position(north + old_north, east + old_east, down + old_down)

# Function to send velocity commands
def send_ned_position(north, east, down):
    """
    Move the drone in NED (North-East-Down) frame.
    north: Positive value means moving north
    east: Positive value means moving east
    down: Positive value means moving down
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      # time_boot_ms (not used)
        0, 0,   # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # NED positions
        0, 0, 0, # NED velocities
        0, 0, 0, # NED accelerations (not supported),
        0, 0)    # yaw, yaw_rate (not supported)

    vehicle.send_mavlink(msg)

    # Target is reached if we are close enough to the target position
    while True:
        remaining = np.array([north, east, down]) - np.array([vehicle.location.local_frame.north, vehicle.location.local_frame.east, vehicle.location.local_frame.down])
        if np.linalg.norm(remaining) < 1:
            print("Target reached")
            break
        time.sleep(0.1)

# Function to change the yaw
def condition_yaw(heading, relative=False):
    """
    Set the yaw angle of the drone.
    heading: Target yaw angle (degrees)
    relative: True for relative yaw change, False for absolute
    """
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Command
        0,       # Confirmation
        heading, # Yaw angle (degrees)
        0,       # Yaw speed (degrees/second)
        is_relative,  # Relative or absolute
        0, 0, 0, 0)  # Params not used
    vehicle.send_mavlink(msg)

    # Target is reached if we are close enough to the target heading
    while True:
        heading_diff = heading - np.rad2deg(vehicle.attitude.yaw)
        if heading_diff > 180:
            heading_diff -= 360
        elif heading_diff < -180:
            heading_diff += 360
        if abs(heading_diff) < 1:
            print("Target heading reached")
            break
        time.sleep(0.1)

# Arm and take off to 10 meters
def arm_and_takeoff(target_altitude):
    """
    Arms the vehicle and flies to a target altitude.
    """
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable...")
        time.sleep(2)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def land():
    """
    Land the vehicle.
    """
    vehicle.mode = VehicleMode("LAND")
    while abs (vehicle.location.global_relative_frame.alt) > 0.2 or not vehicle.armed:
        time.sleep(1)
    print("Landed")

def get_position():
    # Get the pitch (attitude)
    pitch = np.rad2deg( vehicle.attitude.pitch)
    print(f"Pitch: {pitch} radians")

    # Get the position
    north = vehicle.location.local_frame.north
    east = vehicle.location.local_frame.east
    down = vehicle.location.local_frame.down
    print(f"Position: north: {north}, east: {east}, Relative Altitude: {down} meters")

    return north, east, down

# Example usage
try:
    get_position()
    arm_and_takeoff(10)  # Take off to 10 meters
    get_position()
    
    while True:
        inp = input("Enter 'm' to move, 'y' to change yaw, 'exit' to exit: ")
        if inp == "exit":
            break
        elif inp == "m":
            north = float(input("Enter the north value: "))
            east = float(input("Enter the east value: "))
            down = float(input("Enter the down value: "))
            goto_position_new(north, east, down)
            get_position()
        elif inp == "y":
            heading = float(input("Enter the heading value: "))
            condition_yaw(heading)
            get_position()
        else:
            print("Invalid input")

finally:
    print("Landing...")
    land()
    vehicle.close()
