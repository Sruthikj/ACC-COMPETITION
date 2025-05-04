import time
import cv2
import numpy as np
import keyboard
from pal.products.qcar import QCar, QCarRealSense

# Initialize QCar and RealSense Camera
qcar = QCar()
camera = QCarRealSense(mode='RGB, Depth')

# Initialize Webcam for Traffic Light Detection
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Throttle constants (these are the throttle values sent to QCar)
THROTTLE_NORMAL = 0.05   # Forward throttle when distance is clear (≥ 0.06 m)
THROTTLE_REDUCED = 0.03  # Reduced forward throttle when distance is between 0.06 m and 0.03 m
THROTTLE_STOP = 0        # Throttle value to stop the vehicle
THROTTLE_REVERSE = -0.05 # Reverse throttle value

# Distance thresholds in meters
DIST_CLEAR = 0.06  # If distance ≥ 0.06 m, allow full forward throttle (0.05)
DIST_NEAR = 0.03   # If distance is between 0.06 and 0.03 m, use reduced throttle (0.03)
# If distance < 0.03 m, force stop

# Define HSV ranges for traffic light detection (if needed)
red_lower = np.array([136, 87, 111], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)
green_lower = np.array([25, 52, 72], np.uint8)
green_upper = np.array([102, 255, 255], np.uint8)
kernel = np.ones((5, 5), np.uint8)

def get_distance():
    """
    Reads the depth image from the RealSense camera,
    converts from millimeters to meters, and returns the minimum
    distance from a central region.
    """
    camera.read_depth()
    depth_image = camera.imageBufferDepthPX  # Depth in mm
    depth_meters = depth_image.astype(np.float32) * 0.001  # Convert to meters
    center_region = depth_meters[200:280, 280:360]  # Central region of interest
    return np.min(center_region)

def detect_traffic_light():
    """
    Captures a frame from the webcam, applies HSV masks for red/green,
    and returns "RED", "GREEN", or "NONE". Also displays the frame.
    """
    ret, frame = cap.read()
    if not ret:
        return "NONE", None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, red_lower, red_upper)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    red_mask = cv2.dilate(red_mask, kernel)
    green_mask = cv2.dilate(green_mask, kernel)
    
    traffic = "NONE"
    if cv2.countNonZero(red_mask) > 500:
        traffic = "RED"
    elif cv2.countNonZero(green_mask) > 500:
        traffic = "GREEN"
    
    # Display the frame for debugging (optional)
    cv2.imshow("Traffic Detection", frame)
    cv2.waitKey(1)
    
    return traffic, frame

def main():
    print("Starting QCar manual control (using keyboard package)...")
    try:
        while True:
            distance = get_distance()
            traffic_light, _ = detect_traffic_light()
            print(f"DEBUG: Distance = {distance:.3f} m | Traffic Light = {traffic_light}")
            
            # Initialize manual command values
            manual_throttle = 0
            steering = 0

            # Use keyboard package for WASD control
            if keyboard.is_pressed('w'):
                if distance >= DIST_CLEAR:
                    manual_throttle = THROTTLE_NORMAL
                    print("W pressed: Clear, throttle = 0.05")
                elif DIST_NEAR <= distance < DIST_CLEAR:
                    manual_throttle = THROTTLE_REDUCED
                    print("W pressed: Near obstacle, throttle = 0.03")
                else:
                    manual_throttle = THROTTLE_STOP
                    print("W pressed: Obstacle too close, stopping!")
            elif keyboard.is_pressed('s'):
                manual_throttle = THROTTLE_REVERSE
                print("S pressed: Reversing, throttle = -0.05")
            
            if keyboard.is_pressed('a'):
                steering = 0.5
                print("A pressed: Steering left")
            elif keyboard.is_pressed('d'):
                steering = -0.5
                print("D pressed: Steering right")
            
            # Override: if a red traffic light is detected, force stop
            if traffic_light == "RED":
                manual_throttle = THROTTLE_STOP
                print("Red light detected: Forcing stop")
            
            qcar.write(throttle=manual_throttle, steering=steering)
            print(f"Command sent -> Throttle: {manual_throttle}, Steering: {steering}\n")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Stopping QCar...")
        qcar.write(throttle=0, steering=0)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        camera.close()

if __name__ == "__main__":
    main()
