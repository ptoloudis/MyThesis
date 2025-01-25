from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np
from pymavlink import mavutil

class Drone:
    def __init__(self, connection_string):
        self.vehicle = connect(connection_string, wait_ready=True)

    def arm_and_takeoff(self, target_altitude):
        """
        Arms the self.vehicle and flies to a target altitude.
        """
        while not self.vehicle.is_armable:
            print("Waiting for self.vehicle to become armable...")
            time.sleep(2)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
    
    def land(self):
        """
        Land the self.vehicle.
        """
        self.vehicle.mode = VehicleMode("LAND")
        while abs (self.vehicle.location.global_relative_frame.alt) > 0.2 or not self.vehicle.armed:
            time.sleep(1)
        print("Landed")

    def get_position(self):
        # Get the pitch (attitude)
        pitch = np.rad2deg( self.vehicle.attitude.pitch)
        print(f"Pitch: {pitch} radians")

        # Get the position
        north = self.vehicle.location.local_frame.north
        east = self.vehicle.location.local_frame.east
        down = self.vehicle.location.local_frame.down
        print(f"Position: north: {north}, east: {east}, Relative Altitude: {down} meters")

        return north, east, down

    # Function to move the drone to a new position
    def goto_position_new(self, north, east, down):
        old_north, old_east, old_down = self.get_position()
        self.send_ned_position(north + old_north, east + old_east, down + old_down)

    # Function to send velocity commands
    def send_ned_position(self, north, east, down):
        """
        Move the drone in NED (North-East-Down) frame.
        north: Positive value means moving north
        east: Positive value means moving east
        down: Positive value means moving down
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,      # time_boot_ms (not used)
            0, 0,   # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # NED positions
            0, 0, 0, # NED velocities
            0, 0, 0, # NED accelerations (not supported),
            0, 0)    # yaw, yaw_rate (not supported)

        self.vehicle.send_mavlink(msg)

        # Target is reached if we are close enough to the target position
        while True:
            remaining = np.array([north, east, down]) - np.array([self.vehicle.location.local_frame.north, self.vehicle.location.local_frame.east, self.vehicle.location.local_frame.down])
            if np.linalg.norm(remaining) < 1:
                print("Target reached")
                break
            time.sleep(0.1)

    # Function to change the yaw
    def condition_yaw(self, heading, relative=False):
        """
        Set the yaw angle of the drone.
        heading: Target yaw angle (degrees)
        relative: True for relative yaw change, False for absolute
        """
        is_relative = 1 if relative else 0
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Command
            0,       # Confirmation
            heading, # Yaw angle (degrees)
            0,       # Yaw speed (degrees/second)
            is_relative,  # Relative or absolute
            0, 0, 0, 0)  # Params not used
        self.vehicle.send_mavlink(msg)

        # Target is reached if we are close enough to the target heading
        while True:
            heading_diff = heading - np.rad2deg(self.vehicle.attitude.yaw)
            if heading_diff > 180:
                heading_diff -= 360
            elif heading_diff < -180:
                heading_diff += 360
            if abs(heading_diff) < 1:
                print("Target heading reached")
                break
            time.sleep(0.1)

    def close(self):
        self.vehicle.close()
        print("Connection closed")
