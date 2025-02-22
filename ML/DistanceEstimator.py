import cv2
import numpy as np
from ultralytics import YOLO
import math

class DistanceEstimator:
    def __init__(self, cameraHeightM=10, cameraAngleDeg=45, focalLengthMm=37.63, sensorWidthMm=24.89, sensorHeightMm=16.93):
        self.cameraHeight = cameraHeightM
        self.cameraAngle = math.radians(cameraAngleDeg)
        self.focalLength = focalLengthMm
        self.sensorWidth = sensorWidthMm
        self.sensorHeight = sensorHeightMm
        self.averageCarWidth = 1800
        self.averageCarHeight = 1500
        self.model = YOLO('yolo11s.pt')
        
    def calibrateCamera(self, imageWidthPixels, imageHeightPixels):
        self.focalLengthPixels = (self.focalLength * imageWidthPixels) / self.sensorWidth
        self.verticalFov = 2 * math.atan(self.sensorHeight / (2 * self.focalLength))
        self.imageCenterY = imageHeightPixels / 2
        self.pixelsPerVerticalDegree = imageHeightPixels / math.degrees(self.verticalFov)
        
    def calculateBlindSpot(self):
        """Calculate the blind spot triangle dimensions under the camera"""
        # Calculate angle to bottom of FOV
        bottomAngle = self.cameraAngle + self.verticalFov/2
        
        # Calculate blind spot length (distance from camera base to where FOV hits ground)
        blindSpotLength = self.cameraHeight / math.tan(bottomAngle)
        
        # Calculate width of blind spot at ground level
        blindSpotWidth = 2 * blindSpotLength * math.tan(math.radians(self.verticalFov/2))
        
        return {
            'length': blindSpotLength,
            'width': blindSpotWidth,
            'area': 0.5 * blindSpotLength * blindSpotWidth
        }

    def calculateGroundDistance(self, bboxBottomY, imageHeight):
        pixelAngle = math.atan2((bboxBottomY - self.imageCenterY), self.focalLengthPixels)
        actualAngle = self.cameraAngle + pixelAngle
        groundDistance = self.cameraHeight / math.tan(actualAngle)
        
        return groundDistance

    def changeHeightAndAngle(self, newHeight, newAngleDeg):
        self.cameraHeight = newHeight
        self.cameraAngle = math.radians(newAngleDeg)

    def calculateAngle(self, x, y):
        # Calculate the angle in radians
        angleRadians = math.atan2(y - 0, x - 0)
        
        # Convert radians to degrees
        angleDegrees = math.degrees(angleRadians)
        
        return angleDegrees
    
    def pixelToMeter(self, pixelWidth, pixelHeight):
        meterWidth = pixelWidth * self.sensorWidth / 1000 * 2
        meterHeight = pixelHeight * self.sensorHeight / 1000
        return meterWidth, meterHeight
