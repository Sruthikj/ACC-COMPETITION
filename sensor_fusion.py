import cv2
import numpy as np
import time
import keyboard
from pal.products.qcar import QCar

# battery
# color detection
# acc
# tsh
# w s a d
#speed

# Initialize Webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # width
cap.set(4, 480)  # height

# QCar Setup parameters
sampleRate = 200
runTime = 60.0  # seconds

camera = QCarRealSense(mode='RGB, Depth')

# Distance thresholds in meters
DIST_CLEAR = 0.06  # If distance ≥ 0.06 m, allow full forward throttle (0.05)
DIST_NEAR = 0.03   # If distance is between 0.06 and 0.03 m, use reduced throttle (0.03)

# Define HSV color boundaries for red and green
red_lower = np.array([136, 87, 111], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)
green_lower = np.array([25, 52, 72], np.uint8)
green_upper = np.array([102, 255, 255], np.uint8)
kernel = np.ones((5, 5), "uint8")

# Initialize previous color state.
prev_color = "green"

CPR = 2048  # Counts per revolution (example value)
WHEEL_RADIUS = 0.034  # Wheel radius in meters (example value)
SAMPLE_RATE = 200
RUN_TIME = 60.0  # seconds

# Function to calculate speed from encoder counts
def calculate_speed(encoder_counts, time_interval):
    if time_interval == 0:  # Prevent division by zero
        return 0.0
    
    # Calculate rotations
    rotations = encoder_counts / CPR
    # Calculate distance traveled in meters
    distance = rotations * 2 * np.pi * WHEEL_RADIUS
    # Calculate speed in meters per second
    speed_mps = distance / time_interval
    # Convert speed to kilometers per hour
    speed_kmph = speed_mps * 3.6
    return speed_kmph

def get_distance():
    """
    Reads depth image from QCarRealSense and calculates 
    the minimum distance in the center region.
    """
    camera.read_depth()
    depth_image = camera.imageBufferDepthPX  # Depth in pixels

    # Convert depth image to meters
    depth_meters = depth_image.astype(np.float32) * 0.001  # Convert mm to meters

    # Extract center region (80x80 pixels)
    center_region = depth_meters[200:280, 280:360]

    # Get minimum depth value in center region
    min_distance = np.min(center_region)

    return min_distance

with QCar(frequency=sampleRate) as myCar:
    t0 = time.time()

    while time.time() - t0 < runTime:
        ret, frame = cap.read()
        if not ret:
            continue
        #get distamce
        distance = get_distance()

        current_time = time.time()
        time_interval = current_time - last_time

        #get battery voltage
        battery_voltage = myCar.batteryVoltage

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

        current_encoder_count = myCar.motorEncoder[0]

        # Calculate encoder counts difference
        encoder_counts_diff = current_encoder_count - last_encoder_count

        # Calculate speed
        speed_kmph = calculate_speed(encoder_counts_diff, time_interval)

        # Update last time and encoder count
        last_time = current_time
        last_encoder_count = current_encoder_count
        
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
            if keyboard.is_pressed('w'):
                if distance >= DIST_CLEAR:
                    manual_throttle = 0.05
                    print("W pressed: Clear, throttle = 0.05")
                elif DIST_NEAR <= distance < DIST_CLEAR:
                    manual_throttle = 0.03
                    print("W pressed: Near obstacle, throttle = 0.03")
                else:
                    manual_throttle = 0
                    print("W pressed: Obstacle too close, stopping!")
            elif keyboard.is_pressed('s'):
                manual_throttle = -0.05
                print("S pressed: Reversing, throttle = -0.05")
            
            if keyboard.is_pressed('a'):
                steering = 0.5
                print("A pressed: Steering left")
            elif keyboard.is_pressed('d'):
                steering = -0.5
                print("D pressed: Steering right")
                
        # Send control signals to QCar
        myCar.write(throttle, steering, np.array([0, 0, 0, 0, 0, 0, 1, 1]))

        # Display the frame with detected colors
        cv2.imshow("Color Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        print(f'Battery Voltage: {battery_voltage:.2f} V')
        print(f'Speed: {speed_kmph:.2f} km/h')
        
        time.sleep(0.01)  # Small delay to stabilize the loop
        

# Release resources
cap.release()
cv2.destroyAllWindows()
