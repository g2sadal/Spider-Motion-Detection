#!/usr/bin/python3

# Capture a JPEG while still running in the preview mode. When you
# capture to a file, the return value is the metadata for that image.

import time
from picamera2 import Picamera2
import os

save_path = "/home/terradynamics/Desktop/motion/image"
os.makedirs(save_path, exist_ok=True)

picam2_A = Picamera2(0)
picam2_B = Picamera2(1)

capture_config_A = picam2_A.create_still_configuration(main={"size": (4608, 2592)})
picam2_A.configure(capture_config_A)

capture_config_B = picam2_B.create_still_configuration(main={"size": (4608, 2592)})
picam2_B.configure(capture_config_B)

picam2_A.start()
picam2_B.start()
time.sleep(2) 

save_path_A = os.path.join(save_path, "pi_camA.jpg")
save_path_B = os.path.join(save_path, "pi_camB.jpg")

metadata_A = picam2_A.capture_file(save_path_A)
metadata_B = picam2_B.capture_file(save_path_B)

picam2_A.close()
picam2_B.close()
