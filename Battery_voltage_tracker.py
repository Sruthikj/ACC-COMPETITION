import numpy as np
import time
from pal.products.qcar import QCar
import keyboard

# Initial Setup
sampleRate = 200
runTime = 60.0  # seconds

with QCar(frequency=sampleRate) as myCar:
    t0 = time.time()
    
    while time.time() - t0 < runTime:
        t = time.time()

        # Read from onboard sensors
        myCar.read()
        
        # Get battery voltage
        battery_voltage = myCar.batteryVoltage

        # Basic IO - write motor commands
        throttle = 0.0
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])

        if keyboard.is_pressed('W'):
            throttle = 0.05
        elif keyboard.is_pressed('S'):
            throttle = -0.05
        else:
            throttle = 0.0

        if keyboard.is_pressed('A'):
            steering = 0.5
        elif keyboard.is_pressed('D'):
            steering = -0.5
        else:
            steering = 0.0 

        myCar.write(throttle, steering, LEDs)

        # Print battery voltage in real-time
        print(f'Battery Voltage: {battery_voltage:.2f} V')

        time.sleep(0.01)  # Small delay to match the sample rate
