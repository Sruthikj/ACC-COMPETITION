import cv2
import numpy as np
import time
from tensorflow.keras.models import load_model
from pal.products.qcar import QCarRealSense
from qvl.qlabs import QuanserInteractiveLabs

model = load_model("signal_color_model.h5")
class_labels = ["Red", "Yellow", "Green"]

qlabs = QuanserInteractiveLabs()
print("Connecting to QLabs...")
try:
    qlabs.open("localhost")
    print("Connected to QLabs")
except:
    print("Unable to connect to QLabs")
    quit()

camera = QCarRealSense(mode='RGB')

try:
    print("Starting traffic light detection")
    while True:
        camera.read_RGB()
        frame = camera.imageBufferRGB

        image = cv2.resize(frame, (64, 64))
        image = image.astype("float32") / 255.0
        image = np.expand_dims(image, axis=0)

        prediction = model.predict(image, verbose=0)
        predicted_class = class_labels[np.argmax(prediction)]

        display_text = f"{predicted_class} Light"
        color = (0, 255, 0) if predicted_class == "Green" else (0, 0, 255)

        cv2.putText(frame, display_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.imshow("Traffic Light Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    camera.close()
    qlabs.close()
    cv2.destroyAllWindows()
