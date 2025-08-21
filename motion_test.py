import time
import datetime as dt
import argparse
import cv2
import numpy as np
import os
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from datetime import datetime
from collections import deque
import json

def main(enable_preview=False, enable_contour=False):
    '''Create Json file to save the timestamp of each frame of vedio A and B'''
    timestamp_data_A = None
    timestamp_data_B = None
    # define timestamp file
    
    folder_name = datetime.now().strftime("%Y-%m-%d_%H-%M_test")
    folder_path = os.path.join(".", folder_name)
    os.makedirs(folder_path, exist_ok = True)
    
    '''Create a txt file to save the starting time and ending time of each vedio'''
    log_file = "motion_timestamps.txt"
    log_file_path = os.path.join(folder_path, log_file)
    if os.path.exists(log_file_path):
        open(log_file, "w").close()
    
    log_file_B = 'motion_timestamps_B.txt'
    log_file_path_B = os.path.join(folder_path, log_file_B)
    if os.path.exists(log_file_path):
        open(log_file_B, 'w').close()


    '''frame index initialization(for frame allignment)'''
    frame_index_A = 0
    frame_index_B = 0
    
    '''Initialize camera'''
    picam2_A = Picamera2(0)
    picam2_B = Picamera2(1)
    
    img_Size = (4608,2592) # max is 4608x2592, original is 2304x1296, ration is ~2:1

    video_config_A = picam2_A.create_still_configuration(main={"size": img_Size, "format": "RGB888"})
    video_config_B = picam2_B.create_still_configuration(main={"size": img_Size, "format": "RGB888"})

    picam2_A.configure(video_config_A)
    picam2_B.configure(video_config_B)
    
    # number of frames between background updates
    frame_interval = 10 
    counter_A = 0
    counter_B = 0
    motion_threshold = 0.00015

    '''Create a queue to save ten frames before recording'''
    frame_storage_A = deque(maxlen = frame_interval)
    frame_storage_B = deque(maxlen = frame_interval)
    frame_storage_color_A = deque(maxlen = frame_interval)
    frame_storage_color_B = deque(maxlen = frame_interval)
    timestamp_storage_A = deque(maxlen = frame_interval)
    timestamp_storage_B = deque(maxlen = frame_interval)


    '''Initilization'''
    previous_frame = None
    next_frame = None
    recording = False

    previous_frame_B = None
    next_frame_B = None
    recording_B = False

    difference = None
    difference_B = None

    

    # timing for stopping
    start_time = None
    is_timing = False
    
    start_time_B = None
    is_timing_B = False
    
    timeFPS = 1.2
    FPS = 1/timeFPS

    num_vedio = 0   # naming the videos for camera A
    num_vedio_B = 0 # naming the videos for camera B

    # counters for printing whether motion is detected
    print_counter = 0
    print_counter_interval = 1
    print_counter_B = 0
    print_counter_interval_B = 1
    
    frame_index_A = 0
    frame_index_B = 0
    
    writer_A = None
    writer_B = None
    
    picam2_A.start()
    picam2_B.start()
    

    num_vedio = 1
    filename_A = f"{folder_path}/camera_A_{num_vedio}.mp4"
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer_A = cv2.VideoWriter(filename_A, fourcc, FPS, img_Size)
    
    filename_B = f"{folder_path}/camera_B_{num_vedio}.mp4"
    writer_B = cv2.VideoWriter(filename_B, fourcc, FPS, img_Size)
    
    timestamp_data_A = {
        "video_filename": filename_A,
        "start_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),  
        "frames": []
    }
    
    timestamp_data_B = {
        "video_filename": filename_B,
        "start_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),  
        "frames": []
    }
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 3
    font_color = (255, 255, 255)  # White text
    thickness = 5
    line_type = cv2.LINE_AA
    bg_color = (0,0,0)
    padding = 10
    line_spacing = 10
    
    try:
        while True:
            
            trialTime = time.time()
            # Capture current frame
            frame = picam2_A.capture_array()
            sensor_timestamp_A = dt.datetime.now()
            frame_B = picam2_B.capture_array()            
            sensor_timestamp_B = dt.datetime.now()

            if frame.shape[2] != 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            if frame_B.shape[2] != 3:
                frame_B = cv2.cvtColor(frame_B, cv2.COLOR_BGRA2BGR)

            # Convert to grayscale, blur, and threshold
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gausBlur = cv2.GaussianBlur(gray, (5, 5), 0)
            gray_B = cv2.cvtColor(frame_B, cv2.COLOR_BGR2GRAY)
            gausBlur_B = cv2.GaussianBlur(gray_B, (5, 5), 0)
            
            frame_storage_A.append(gausBlur)
            frame_storage_B.append(gausBlur_B)
            frame_storage_color_A.append(frame)
            frame_storage_color_B.append(frame_B)
            timestamp_storage_A.append(sensor_timestamp_A)
            timestamp_storage_B.append(sensor_timestamp_B)
            
            if len(frame_storage_A) < frame_interval or len(frame_storage_B) < frame_interval:
                pass
            else:
                previous_frame = frame_storage_A[0]
                previous_frame_B = frame_storage_B[0]

                counter_A += 1
                counter_B += 1

                # Compute absolute difference
                difference = cv2.absdiff(gausBlur, previous_frame)
                difference_B = cv2.absdiff(gausBlur_B, previous_frame_B)

                # Convert to grayscale, blur, and threshold
                _, binaryImage = cv2.threshold(difference, 20, 255, cv2.THRESH_BINARY)
                _, binaryImage_B = cv2.threshold(difference_B, 20, 255, cv2.THRESH_BINARY)

                # 2 ways(Considering the GPU performance)

                white_A = cv2.countNonZero(binaryImage)
                dilated_A = cv2.dilate(binaryImage, None, iterations=2)
                cnts_A, _ = cv2.findContours(dilated_A.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                for c in cnts_A:
                    if cv2.contourArea(c) > 100:
                        contour_A = True
                        break
                        
                white_B = cv2.countNonZero(binaryImage_B)
                dilated_B = cv2.dilate(binaryImage_B, None, iterations=2)
                cnts_B, _ = cv2.findContours(dilated_B.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for c_B in cnts_B:
                    if cv2.contourArea(c_B) > 100:
                        contour_B = True
                        break
                
                motion_detected = (white_A > 20) and contour_A
                motion_detected_B = (white_B > 20) and contour_B
                
                '''mark where is moving when enable_contour is true'''
                
                

                print("Motion detected(A):", motion_detected)
                print("Motion detected(B):", motion_detected_B)
                
                timestamp_data_A["frames"].append({
                    "frame_index": frame_index_A,
                    "timestamp": str(sensor_timestamp_A),
                    "motion value": white_A,
                    "motion threshold": int(gray.size * 0.005)
                })
                frame_index_A += 1
                
                timestamp_data_B["frames"].append({
                    "frame_index": frame_index_B,
                    "timestamp": str(sensor_timestamp_B),
                    "motion value": white_B,
                    "motion threshold": int(gray.size * 0.005)
                })
                frame_index_B += 1
                
                # Define font settings
                bool_bg_A = (0,128,0) if motion_detected else (0,0,255)
                bool_bg_B = (0,128,0) if motion_detected_B else (0,0,255)
                
                # Add text to image
                (text_width, text_height), baseline = cv2.getTextSize(str(sensor_timestamp_A) + " Frame: " + str(frame_index_A), font, font_scale, thickness)
                x,y1 = padding, padding + text_height
                
                (bool_width, bool_height), bool_baseline = cv2.getTextSize(str(100000), font, font_scale, thickness)
                y2 = y1 + bool_height + line_spacing
                
                frame = cv2.rectangle(frame, (x-padding, y1-text_height-padding), (x + text_width + padding, y1 + baseline + padding), bg_color, cv2.FILLED)
                frame_B = cv2.rectangle(frame_B, (x-padding, y1-text_height-padding), (x + text_width + padding, y1 + baseline + padding), bg_color, cv2.FILLED)
                frame = cv2.rectangle(frame, (x - padding, y2 - bool_height - padding), (x + bool_width + padding, y2 + bool_baseline + padding), bg_color, cv2.FILLED)
                frame_B = cv2.rectangle(frame_B, (x - padding, y2 - bool_height - padding), (x + bool_width + padding, y2 + bool_baseline + padding), bg_color, cv2.FILLED)
                
                
                frame = cv2.putText(frame, str(sensor_timestamp_A) + " Frame: " + str(frame_index_A), (x, y1), font, font_scale, font_color, thickness, line_type)
                frame_B = cv2.putText(frame_B, str(sensor_timestamp_B) + " Frame: " + str(frame_index_B), (x, y1), font, font_scale, font_color, thickness, line_type)
                frame = cv2.putText(frame, str(white_A), (x, y2), font, font_scale, bool_bg_A, thickness, line_type)
                frame_B = cv2.putText(frame_B, str(white_B), (x, y2), font, font_scale, bool_bg_B, thickness, line_type)
                
                writer_A.write(frame)
                writer_B.write(frame_B)
                
                if enable_preview:
                    if difference is not None:
                        color_diff_A = cv2.cvtColor(difference, cv2.COLOR_GRAY2BGR)
                    else:
                        color_diff_A = frame  
                    
                    if difference_B is not None:
                        color_diff_B = cv2.cvtColor(difference_B, cv2.COLOR_GRAY2BGR)
                    else:
                        color_diff_B = frame_B  

                    preview_A = np.hstack((color_diff_A, frame))
                    preview_B = np.hstack((color_diff_B, frame_B))
                    preview = np.vstack((preview_A, preview_B))
                    #cv2.imshow("Camera A (preview)", preview_A)
                    #cv2.imshow("Camera B (preview)", preview_B)
                    preview_resized = cv2.resize(preview, (0,0), fx = 0.2, fy = 0.2)
                    cv2.imshow("Camera_preview(A and B)", preview_resized)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            
            
            trialTime1 = trialTime
            endTime = time.time()
            if endTime-trialTime1 < timeFPS:
                time.sleep(timeFPS - endTime + trialTime)
                
        stop_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        with open(log_file_path, "a") as f:
            f.write(f"Video {num_vedio} (File: {folder_path}/camera_A_{num_vedio}.h264): Stop at {stop_timestamp}\n")

        if timestamp_data_A is not None:
            timestamp_data_A["stop_time"] = stop_timestamp
            
            timestamp_json_file = f"camera_A_{num_vedio}_timestamp.json"
            timestamp_json_file_path = os.path.join(folder_path, timestamp_json_file)
            
            with open(timestamp_json_file_path, "w", encoding="utf-8") as jf:
                json.dump(timestamp_data_A, jf, indent=4)
                
        if timestamp_data_B is not None:
            timestamp_data_B["stop_time"] = stop_timestamp
            
            timestamp_json_file = f"camera_B_{num_vedio}_timestamp.json"
            timestamp_json_file_path = os.path.join(folder_path, timestamp_json_file)
            
            with open(timestamp_json_file_path, "w", encoding="utf-8") as jf:
                json.dump(timestamp_data_B, jf, indent=4)

            timestamp_data_B = None

    finally:
        if writer_A is not None:
            writer_A.release()
        if writer_B is not None:
            writer_B.release()
        picam2_A.stop()
        picam2_B.stop()
        cv2.destroyAllWindows()


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

    main(enable_preview=args.enable_preview, enable_contour=args.enable_contour)


