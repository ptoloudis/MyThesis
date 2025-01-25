import torch
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
import DistanceEstimator
import math
import DroneMovement

# URL of the live stream page
live_stream_url = "http://127.0.0.1:8080/MyDemoLive/index.html"

def capture_frame_from_video(video_element):
    """Capture a frame from the video element."""
    try:
        # Capture a screenshot from the video element using Selenium
        video_screenshot = video_element.screenshot_as_png
        np_array = np.frombuffer(video_screenshot, np.uint8)
        frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        return frame
    except Exception as e:
        print(f"Error capturing frame: {e}")
        return None

def process_image(image, estimator):
    estimator.calibrate_camera(image.shape[1], image.shape[0])
    blind_spot = estimator.calculate_blind_spot()
    results = estimator.model(image)
    annotated_image = image.copy()

    distances = {}

    for result in results[0].boxes.data:
        x1, y1, x2, y2, conf, cls = result
        
        if int(cls) == 2:  # car class
            ground_distance = estimator.calculate_ground_distance(y2, image.shape[0])
            
            cv2.rectangle(annotated_image, 
                         (int(x1), int(y1)), 
                         (int(x2), int(y2)), 
                         (0, 255, 0), 
                         2)
            
            #  Extract confidence and class name
            conf_text = f"{conf:.2f}"  # Format confidence to 2 decimal places
            class_name = estimator.model.names[int(cls)]  # Map class index to class name

            # Add distance labels
            label = f'{class_name} {conf_text} {ground_distance:.2f}m'
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            label_y = max(y1, label_size[1] + 10)
            cv2.rectangle(annotated_image, (int(x1), int(label_y - label_size[1] - 10)), (int(x1 + label_size[0]), int(label_y + 5)), (0, 255, 0), -1)  # Filled rectangle for label background
            cv2.putText(annotated_image, label, (int(x1), int(label_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
            
            # Draw line showing distance
            center_x = int((x1 + x2) / 2)
            cv2.line(annotated_image,
                    (center_x, int(y2)),
                    (int(image.shape[1]/2), image.shape[0]),
                    (0, 0, 255),
                    2)
            
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            cv2.line(annotated_image,
                    (center_x, int(y2)),
                    (int(image.shape[1]/2), image.shape[0]),
                    (0, 0, 255),
                    2)
            
            # Distance in pixels and go to meters
            distance_y = (center_x - image.shape[1]/2) * estimator.sensor_width/ 1000 * 2

            # Total distance 
            total_distance = math.sqrt(distance_y**2 + ground_distance**2)

            # Angle between the points
            angle = estimator.calculate_angle(ground_distance, distance_y)

            distances = {
                'distance_y': distance_y,
                'ground_distance': ground_distance, 
                'total_distance': total_distance,
                'blind_spot': blind_spot['length'],
                'center_x': center_x,
                'center_y': center_y
            }

    return annotated_image, distances

def main():
    diff = [0, 0]
    car_location = [0, 0]

    # Load the YOLO model
    estimator = DistanceEstimator.DistanceEstimator()
    print("Model loaded successfully")
    
    # Set up Chrome options
    chrome_options = Options()
    chrome_options.add_argument("--headless")  # Run in headless mode
    # chrome_options.add_argument("--window-size=1920,1080")  # Set window size to 1920x1080

    # Initialize WebDriver using WebDriver Manager (No need to specify version this way)
    service = Service(ChromeDriverManager().install())  # Automatically picks the correct version
    driver = webdriver.Chrome(service=service, options=chrome_options)

    try:
        # Open the live stream page
        driver.get(live_stream_url)

        # Wait for the play button to appear dynamically
        play_button = WebDriverWait(driver, 20).until(
            EC.presence_of_element_located((By.ID, "playButton"))
        )

        # Click the play button
        play_button.click()

        time.sleep(2)

        print("Live stream started")
        while True:
            
            start_time = time.time()

            # Wait for the video element to be added dynamically
            video_element = WebDriverWait(driver, 20).until(
                EC.presence_of_element_located((By.ID, "Video"))
            )

            frame = capture_frame_from_video(video_element)

            if frame is not None:
                # Change the heigth and the angle of the camera
                estimator.change_height_and_angle(10, 0)

                # Run YOLO object detection on the captured frame
                result_image = process_image(frame, estimator)
                
                if result_image[1] == {}:
                    continue

                # Draw a point at the car's location
                car_location_new = result_image[1]['center_x'], result_image[1]['center_y']
                cv2.circle(result_image[0], car_location_new, radius=2, color=(0, 0, 255), thickness=-1)
                cv2.circle(result_image[0], car_location, radius=2, color=(0, 0, 255), thickness=-1)

                print("Distances: ", result_image[1])

                diff[0] = car_location_new[0] - car_location[0]
                diff[1] = car_location_new[1] - car_location[1]
                car_location = car_location_new
                car_velocity = estimator.pixel_to_meter(diff[0], diff[1])

                print(f"Car velocity {car_velocity} m/s")

                finish_time = time.time()
                print(f"Time: {finish_time - start_time} seconds")

                cv2.imshow('Distance and Height Estimation', result_image[0])
                cv2.waitKey(0)
                print("Image processed successfully")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        cv2.destroyAllWindows()
        driver.quit()

if __name__ == "__main__":
    main()