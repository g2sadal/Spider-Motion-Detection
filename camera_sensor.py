import time
import argparse
import cv2
import numpy as np
import os
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from datetime import datetime
from collections import deque
import serial

   


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Dual camera motion detection with optional preview and contour"
    )
    parser.add_argument(
        "--enable_preview",
        action="store_true",
        help="Enable preview windows."
    )
    parser.add_argument(
        "--enable_contour",
        action="store_true",
        help="Use contour detection (and draw rectangles)."
    )
    args = parser.parse_args()
    
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.reset_input_buffer()
    
    enable_preview=args.enable_preview
    enable_contour=args.enable_contour
    
    # Initialize camera
    picam2_A = Picamera2(0)

    still_config_A = picam2_A.create_still_configuration(main={"size": (500, 500), "format": "RGB888"}, controls={"FrameRate": 50})

    picam2_A.configure(still_config_A)
    
    frame_storage_A = deque(maxlen = 10)
    frame_storage_color_A = deque(maxlen = 10)

    previous_frame = None
    next_frame = None

    # number of frames between background updates
    frame_interval = 10  
    counter_A = 0
    
    picam2_A.start()
    
    while True:
        # set motion detected as False at first
        motion_detected = False

        # Capture current frame
        frame = picam2_A.capture_array()
        metadata_A = picam2_A.capture_metadata()

        if frame.shape[2] != 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        # Convert to grayscale, blur, and threshold
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gausBlur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        frame_storage_A.append(gausBlur)
        frame_storage_color_A.append(frame)
        
        if len(frame_storage_A) < frame_interval:
            print("first 10")
            continue

        previous_frame = frame_storage_A[0]

        counter_A += 1

        # Compute absolute difference
        difference = cv2.absdiff(gausBlur, previous_frame)

        # Convert to grayscale, blur, and threshold
        _, binaryImage = cv2.threshold(difference, 20, 255, cv2.THRESH_BINARY)


        white_A = cv2.countNonZero(binaryImage)
        motion_detected = white_A > int(gray.size * 0.0005)
        
        if motion_detected:
            ser.write(b"1\n")
            print("Motion Detected: True")
        else:
            ser.write(b"0\n")            
            print("Motion Detected: False")
            
        if enable_preview:
            color_diff_A = cv2.cvtColor(difference, cv2.COLOR_GRAY2BGR)

            preview_A = np.hstack((color_diff_A, frame))
            preview_resized = cv2.resize(preview_A, (0,0), fx = 0.5, fy = 0.5)
            cv2.imshow("Camera A (preview)", preview_resized)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    
    
    picam2_A.stop()
    cv2.destroyAllWindows()        
        

