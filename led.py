import numpy as np
import time
from pal.products.qcar import QCar
import keyboard

# Initial Setup
sampleRate = 200
runTime = 60.0  # seconds
BLINK_INTERVAL = 0.5  # Time in seconds between blinks

with QCar(frequency=sampleRate) as myCar:
    t0 = time.time()
    led_state = 0
    last_blink_time = time.time()

    while time.time() - t0 < runTime:
        t = time.time()

        # Read from onboard sensors
        myCar.read()

        # Default states
        throttle = 0.0
        steering = 0.0
        LEDs = np.zeros(8, dtype=int)  # Create an 8-element array with all LEDs OFF

        # Movement Controls
        if keyboard.is_pressed('w'):  # Forward
            throttle = 0.05
        elif keyboard.is_pressed('space'):  # Brake
            throttle = -0.05
            # Blinking logic for brake lights
            if time.time() - last_blink_time >= BLINK_INTERVAL:
                led_state = 1 - led_state  # Toggle LED state
                last_blink_time = time.time()

            LEDs[1] = led_state  # Rear brake light 1
            LEDs[2] = led_state  # Rear brake light 2
        else:
            throttle = 0.0
            LEDs[1] = 0  # Ensure brake LEDs turn off when not braking
            LEDs[2] = 0

        # Steering Controls
        if keyboard.is_pressed('a'):  # Turn Left
            steering = 0.5
        elif keyboard.is_pressed('d'):  # Turn Right
            steering = -0.5
        else:
            steering = 0.0

        # Send updated values to QCar
        myCar.write(throttle=throttle, steering=steering, LEDs=LEDs)

        print(f'Throttle: {throttle}, Steering: {steering}, LEDs: {LEDs}')

        time.sleep(0.01)  # Maintain loop timing

print("LED blinking completed!")
