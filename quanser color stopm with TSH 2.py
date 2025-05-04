import cv2
import numpy as np
import time
import keyboard
from pal.products.qcar import QCar

# Initialize Webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # width
cap.set(4, 480)  # height

# QCar Setup parameters
sampleRate = 200
runTime = 60.0  # seconds

# Define HSV color boundaries for red and green
red_lower = np.array([136, 87, 111], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)
green_lower = np.array([25, 52, 72], np.uint8)
green_upper = np.array([102, 255, 255], np.uint8)
kernel = np.ones((5, 5), "uint8")

# Initialize previous color state.
prev_color = "green"

with QCar(frequency=sampleRate) as myCar:
    t0 = time.time()
    while time.time() - t0 < runTime:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Convert frame from BGR to HSV for color detection
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create masks for red and green colors
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
        
        # Dilate masks to reduce noise
        red_mask = cv2.dilate(red_mask, kernel)
        green_mask = cv2.dilate(green_mask, kernel)
        
        # Find contours for red and green
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        red_detected, green_detected = False, False
        
        # Check for red color contours
        for contour in red_contours:
            if cv2.contourArea(contour) > 300:
                red_detected = True
                break  # Stop after detecting a significant red area
        
        # Check for green color contours (only if red was not detected)
        if not red_detected:
            for contour in green_contours:
                if cv2.contourArea(contour) > 300:
                    green_detected = True
                    break
        
        # Update color state
        current_color = "red" if red_detected else ("green" if green_detected else prev_color)
        prev_color = current_color  # Store state for next iteration

        # Initialize throttle and steering
        throttle, steering = 0.0, 0.0
        
        # Car control logic
        if current_color == "red":
            throttle, steering = 0.0, 0.0
        elif current_color == "green":
            if keyboard.is_pressed('W'):
                throttle = 0.03 if (keyboard.is_pressed('A') or keyboard.is_pressed('D')) else 0.05
                steering = 0.5 if keyboard.is_pressed('A') else (-0.5 if keyboard.is_pressed('D') else 0.0)
            elif keyboard.is_pressed('S'):
                throttle = -0.03 if (keyboard.is_pressed('A') or keyboard.is_pressed('D')) else -0.05
                steering = 0.5 if keyboard.is_pressed('A') else (-0.5 if keyboard.is_pressed('D') else 0.0)
            elif keyboard.is_pressed('A'):
                steering = 0.5
            elif keyboard.is_pressed('D'):
                steering = -0.5
        
        # Send control signals to QCar
        myCar.write(throttle, steering, np.array([0, 0, 0, 0, 0, 0, 1, 1]))

        # Display the frame with detected colors
        cv2.imshow("Color Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        time.sleep(0.01)  # Small delay to stabilize the loop

# Release resources
cap.release()
cv2.destroyAllWindows()
