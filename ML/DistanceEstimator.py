import cv2
import numpy as np
from ultralytics import YOLO
import math

class DistanceEstimator:
    def __init__(self, camera_height_m=10, camera_angle_deg=45, focal_length_mm=37.63, sensor_width_mm=24.89, sensor_height_mm=16.93):
        self.camera_height = camera_height_m
        self.camera_angle = math.radians(camera_angle_deg)
        self.focal_length = focal_length_mm
        self.sensor_width = sensor_width_mm
        self.sensor_height = sensor_height_mm
        self.average_car_width = 1800
        self.average_car_height = 1500
        self.model = YOLO('yolo11s.pt')
        
    def calibrate_camera(self, image_width_pixels, image_height_pixels):
        self.focal_length_pixels = (self.focal_length * image_width_pixels) / self.sensor_width
        self.vertical_fov = 2 * math.atan(self.sensor_height / (2 * self.focal_length))
        self.image_center_y = image_height_pixels / 2
        self.pixels_per_vertical_degree = image_height_pixels / math.degrees(self.vertical_fov)
        
    def calculate_blind_spot(self):
        """Calculate the blind spot triangle dimensions under the camera"""
        # Calculate angle to bottom of FOV
        bottom_angle = self.camera_angle + self.vertical_fov/2
        
        # Calculate blind spot length (distance from camera base to where FOV hits ground)
        blind_spot_length = self.camera_height / math.tan(bottom_angle)
        
        # Calculate width of blind spot at ground level
        blind_spot_width = 2 * blind_spot_length * math.tan(math.radians(self.vertical_fov/2))
        
        return {
            'length': blind_spot_length,
            'width': blind_spot_width,
            'area': 0.5 * blind_spot_length * blind_spot_width
        }

    def calculate_ground_distance(self, bbox_bottom_y, image_height):
        pixel_angle = math.atan2((bbox_bottom_y - self.image_center_y), self.focal_length_pixels)
        actual_angle = self.camera_angle + pixel_angle
        ground_distance = self.camera_height / math.tan(actual_angle)
        
        return ground_distance

    def change_height_and_angle(self, new_height_m, new_angle_deg):
        self.camera_height = new_height_m
        self.camera_angle = math.radians(new_angle_deg)

    def calculate_angle(self, x, y):
        # Calculate the angle in radians
        angle_radians = math.atan2(y - 0, x - 0)
        
        # Convert radians to degrees
        angle_degrees = math.degrees(angle_radians)
        
        return angle_degrees
    
    def pixel_to_meter(self, pixel_width, pixel_height):
        meter_width = pixel_width * self.sensor_width / 1000 * 2
        meter_height = pixel_height * self.sensor_height / 1000
        return meter_width, meter_height