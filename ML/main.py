import cv2
import numpy as np
from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from webdriver_manager.chrome import ChromeDriverManager
import time
import math

# My Libraries
import DistanceEstimator
import DroneMovement

# Theshold for the distance between the car and the center of the image
threshold = 15 # in meters
thresholdY = 5 # in meters
thresholdTime = 30 # in seconds
yawChange = 20 # in degrees

# Url for the Drone Movement
droneMovementUrl = '192.168.0.5:14549'

# URL of the live stream page
liveStreamUrl = "http://127.0.0.1:8080/MyDemoLive/index.html"

def captureFrameFromVideo(videoElement):
    """Capture a frame from the video element."""
    try:
        # Capture a screenshot from the video element using Selenium
        videoScreenshot = videoElement.screenshot_as_png
        npArray = np.frombuffer(videoScreenshot, np.uint8)
        frame = cv2.imdecode(npArray, cv2.IMREAD_COLOR)
        return frame
    except Exception as e:
        print(f"Error capturing frame: {e}")
        return None

def processImage(image, estimator):
    estimator.calibrateCamera(image.shape[1], image.shape[0])
    blindSpot = estimator.calculateBlindSpot()
    results = estimator.model.predict(image, conf=0.6)
    annotatedImage = image.copy()

    distances = {}

    for result in results[0].boxes.data:
        x1, y1, x2, y2, conf, cls = result
        
        if int(cls) == 2:  # car class
            groundDistance = estimator.calculateGroundDistance(y2, image.shape[0])
            
            cv2.rectangle(annotatedImage, 
                         (int(x1), int(y1)), 
                         (int(x2), int(y2)), 
                         (0, 255, 0), 
                         2)
            
            #  Extract confidence and class name
            confText = f"{conf:.2f}"  # Format confidence to 2 decimal places
            className = estimator.model.names[int(cls)]  # Map class index to class name

            # Add distance labels
            label = f'{className} {confText} {groundDistance:.2f}m'
            labelSize, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            labelY = max(y1, labelSize[1] + 10)
            cv2.rectangle(annotatedImage, (int(x1), int(labelY - labelSize[1] - 10)), (int(x1 + labelSize[0]), int(labelY + 5)), (0, 255, 0), -1)  # Filled rectangle for label background
            cv2.putText(annotatedImage, label, (int(x1), int(labelY)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
            
            # Debugging: Print values of x1, x2, and centerX
            print(f"x1: {x1}, x2: {x2}")

            # Draw line showing distance
            centerX = int((x1 + x2) / 2)
            centerY = int((y1 + y2) / 2)
            print(f"centerX: {centerX}, centerY: {centerY}")  # Debugging: Print values of centerX and centerY
            cv2.line(annotatedImage,
                    (centerX, int(y2)),
                    (int(image.shape[1]/2), image.shape[0]),
                    (0, 0, 255),
                    2)
            
            # Distance in pixels and go to meters
            distanceX = (centerX - image.shape[1]/2) * estimator.sensorWidth/ 1000 * 2

            # Total distance 
            totalDistance = math.sqrt(distanceX**2 + groundDistance**2)

            # Angle between the points
            angle = estimator.calculateAngle(groundDistance, distanceX)

            distances = {
                'distanceX': distanceX,
                'groundDistance': groundDistance, 
                'totalDistance': totalDistance,
                'blindSpot': blindSpot['length'],
                'centerX': centerX,
                'centerY': centerY
            }

    return annotatedImage, distances

def main():
    timeNotDetect = None
    diff = [0, 0]
    carLocation = [0, 0]

    # Connect to the Drone Movement
    droneMovement = DroneMovement.DroneMovement(droneMovementUrl)
    print("Drone Movement connected successfully")

    # Load the YOLO model
    estimator = DistanceEstimator.DistanceEstimator()
    print("Model loaded successfully")
    
    # Set up Chrome options
    chromeOptions = Options()
    chromeOptions.add_argument("--headless")  # Run in headless mode
    # chromeOptions.add_argument("--window-size=1920,1080")  # Set window size to 1920x1080

    # Initialize WebDriver using WebDriver Manager (No need to specify version this way)
    service = Service(ChromeDriverManager().install())  # Automatically picks the correct version
    driver = webdriver.Chrome(service=service, options=chromeOptions)
    print("WebDriver initialized successfully")

    try:
        # Open the live stream page
        driver.get(liveStreamUrl)

        # Wait for the play button to appear dynamically
        playButton = WebDriverWait(driver, 20).until(
            EC.presence_of_element_located((By.ID, "playButton"))
        )

        # Click the play button
        playButton.click()

        time.sleep(2)

        print("Live stream started")
        while True:
            
            # Wait for the video element to be added dynamically
            videoElement = WebDriverWait(driver, 20).until(
                EC.presence_of_element_located((By.ID, "Video"))
            )

            frame = captureFrameFromVideo(videoElement)

            if frame is not None:
                # Change the heigth and the angle of the camera
                _,_,alt,angle = droneMovement.getPosition()
                estimator.changeHeightAndAngle(-alt, angle)

                # Run YOLO object detection on the captured frame
                resultImage = processImage(frame, estimator)
                
                if resultImage[1] == {}:
                    if timeNotDetect is None:
                        timeNotDetect = time.time()
                    elif time.time() - timeNotDetect > thresholdTime:
                        print(f"Car not detected for {thresholdTime} seconds")
                        yawChangeNew = yawChange + droneMovement.getYaw()
                        yawChangeNew = yawChangeNew if yawChangeNew < 360 else yawChangeNew - 360
                        droneMovement.conditionYaw(yawChangeNew, relative=True)
                        time.sleep(10)
                        timeNotDetect = time.time()
                    
                    continue
                
                timeNotDetect = None

                # Draw a point at the car's location
                carLocationNew = resultImage[1]['centerX'], resultImage[1]['centerY']
                cv2.circle(resultImage[0], carLocationNew, radius=2, color=(0, 0, 255), thickness=-1)
                cv2.circle(resultImage[0], carLocation, radius=2, color=(0, 0, 255), thickness=-1)

                diff[0] = carLocationNew[0] - carLocation[0]
                diff[1] = carLocationNew[1] - carLocation[1]
                carLocation = carLocationNew
                carVelocity = estimator.pixelToMeter(diff[0], diff[1])

                print("Distances: ", resultImage[1])
                print(f"Car velocity {carVelocity} m/s")


                # cv2.imshow('Distance and Height Estimation', resultImage[0])
                # cv2.waitKey(0)
                print("Image processed successfully")

                # Move the drone to the new position
                moveDistance = resultImage[1]['groundDistance'] - resultImage[1]['blindSpot'] 
                if abs(moveDistance) > threshold:
                    newX = moveDistance - threshold
                else:
                    newX = 0
                
                if abs(resultImage[1]['distanceX']) > thresholdY:
                    newY = resultImage[1]['distanceX']
                else:
                    newY = 0

                print(f"Moving to new position: {newX}, {newY}")
                droneMovement.gotoPositionNew(newX, newY, 0)

                time.sleep(15)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        droneMovement.close()
        cv2.destroyAllWindows()
        driver.quit()

if __name__ == "__main__":
    main()