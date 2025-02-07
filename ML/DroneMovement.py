from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np
from pymavlink import mavutil
import math

class DroneMovement:
    def __init__(self, connectionString):
        self.vehicle = connect(connectionString, wait_ready=True)
        print(f"#Connected to vehicle {connectionString}")

    def armAndTakeoff(self, targetAltitude):
        """
        Arms the vehicle and flies to a target altitude.
        """
        while not self.vehicle.is_armable:
            print("#Waiting for vehicle to become armable...")
            time.sleep(2)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("#Waiting for arming...")
            time.sleep(1)

        print("#Taking off!")
        self.vehicle.simple_takeoff(targetAltitude)

        while True:
            if self.vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
                print("#Reached target altitude")
                break
            time.sleep(1)
    
    def land(self):
        """
        Land the vehicle.
        """
        self.vehicle.mode = VehicleMode("LAND")
        while abs(self.vehicle.location.global_relative_frame.alt) > 0.2 or not self.vehicle.armed:
            time.sleep(1)
        print("#Landed")

    def getPosition(self):
        # Get the pitch (attitude)
        pitch = np.rad2deg(self.vehicle.attitude.pitch)

        # Get the position
        north = self.vehicle.location.local_frame.north
        east = self.vehicle.location.local_frame.east
        down = self.vehicle.location.local_frame.down

        return north, east, down, pitch

    # Function to move the drone to a new position
    def gotoPositionNew(self, newX, newY, down):
        oldNorth, oldEast, oldDown, _ = self.getPosition()
        yaw = self.getYaw()  # Get yaw in degrees

        # Convert yaw to radians
        yaw_rad = math.radians(yaw)

        # Rotate the newX and newY using the yaw angle
        rotatedX = newX * math.cos(yaw_rad) - newY * math.sin(yaw_rad)
        rotatedY = newX * math.sin(yaw_rad) + newY * math.cos(yaw_rad)

        # Compute new position
        newNorth = oldNorth + rotatedX
        newEast = oldEast + rotatedY
        newDown = oldDown + down  # Assuming down is in the same frame

        # Send the new position
        self.sendNedPosition(newNorth, newEast, newDown)

    # Function to send velocity commands
    def sendNedPosition(self, north, east, down):
        """
        Move the drone in NED (North-East-Down) frame.
        north: Positive value means moving north
        east: Positive value means moving east
        down: Positive value means moving down
        """
        # Get the current yaw angle
        currentYaw = self.vehicle.attitude.yaw

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,      # time_boot_ms (not used)
            0, 0,   # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000 & ~0b0000011000000000,  # Enable yaw control
            north, east, down, # NED positions
            0, 0, 0, # NED velocities
            0, 0, 0, # NED accelerations (not supported),
            currentYaw, 0)    # yaw, yaw_rate 

        self.vehicle.send_mavlink(msg)

        # Target is reached if we are close enough to the target position
        while True:
            remaining = np.array([north, east, down]) - np.array([self.vehicle.location.local_frame.north, self.vehicle.location.local_frame.east, self.vehicle.location.local_frame.down])
            if np.linalg.norm(remaining) < 1:
                print("#Target reached")
                break
            time.sleep(0.1)

    def getYaw(self):
        return np.rad2deg(self.vehicle.attitude.yaw)

    # Function to change the yaw
    def conditionYaw(self, heading, relative=False):
        """
        Set the yaw angle of the drone.
        heading: Target yaw angle (degrees)
        relative: True for relative yaw change, False for absolute
        """
        isRelative = 1 if relative else 0
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # Command
            0,       # Confirmation
            heading, # Yaw angle (degrees)
            0,       # Yaw speed (degrees/second)
            isRelative,  # Relative or absolute
            0, 0, 0, 0)  # Params not used
        self.vehicle.send_mavlink(msg)

        # Target is reached if we are close enough to the target heading
        while True:
            headingDiff = heading - np.rad2deg(self.vehicle.attitude.yaw)
            if headingDiff > 180:
                headingDiff -= 360
            elif headingDiff < -180:
                headingDiff += 360
            if abs(headingDiff) < 1:
                print("#Target heading reached")
                break
            time.sleep(0.1)

    def close(self):
        self.vehicle.close()
        print("#Connection closed")
