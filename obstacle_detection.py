import time
import cv2
import numpy as np
from pal.products.qcar import QCar, QCarRealSense
import keyboard

# Initialize Virtual QCar and RealSense Camera
qcar = QCar()
camera = QCarRealSense(mode='RGB, Depth')

# Throttle and Obstacle Detection Parameters
THROTTLE_SPEED = 0.04  # Speed when no obstacle
BRAKE_DISTANCE = 0.04   # Stop if obstacle ≤ 1 meter

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

def main():
    print("Starting Virtual QCar...")

    try:
        
        while True:
            throttle=0.0
            steering=0.0
            
            distance = get_distance()
            print(distance)
            if keyboard.is_pressed('W'):
                throttle = 0.5
                print(f"Obstacle Distance: {distance:.2f} meters")
                if distance <= BRAKE_DISTANCE:
                    qcar.write(throttle=0, steering=0)  # Stop the car
                    print("Obstacle detected! Braking...")
            elif keyboard.is_pressed('S'):
                throttle = -0.05
    
            if keyboard.is_pressed('A'):
                steering = -0.5
            elif keyboard.is_pressed('D'):
                steering = 0.5 
            else:
                steering = 0.0

            qcar.write(throttle=throttle, steering=steering)  

            time.sleep(0.1)  

    except KeyboardInterrupt:
        print("Stopping QCar...")
        qcar.write(throttle=0, steering=0)  # Stop the car
        camera.close()

if __name__ == "__main__":
    main()
