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

def main(enable_preview=False, enable_contour=False, frame_interval=10, resolution="4608x2592", timeFPS=1.2, delay=2, motion_threshold=0.005):
    try:
        w, h = map(int, resolution.lower().split('x'))
        img_Size = (w, h)
    except ValueError:
        raise ValueError("--resolution has to be “width x height”, such as 2304x1296")
    
    '''Create Json file to save the timestamp of each frame of vedio A and B'''
    timestamp_data_A = None
    timestamp_data_B = None
    # define timestamp file
    
    base_dir = os.path.dirname(os.path.abspath(__file__))  
    folder_name = datetime.now().strftime("%Y-%m-%d_%H-%M_test")
    folder_path = os.path.join(base_dir, folder_name)
    os.makedirs(folder_path, exist_ok=True)
    
    '''Create a txt file to save the starting time and ending time of each vedio'''
    log_file = "motion_timestamps.txt"
    log_file_path = os.path.join(folder_path, log_file)
    # if os.path.exists(log_file_path):
    #     open(log_file, "w").close()
    
    if os.path.exists(log_file_path):
        open(log_file_path, "w").close()

    log_file_B = 'motion_timestamps_B.txt'
    log_file_path_B = os.path.join(folder_path, log_file_B)
    # if os.path.exists(log_file_path):
    #     open(log_file_B, 'w').close()
    if os.path.exists(log_file_path):
        open(log_file_path_B, 'w').close()
        
    '''frame index initialization(for frame allignment)'''
    frame_index_A = 0
    frame_index_B = 0
    

    '''Create a queue to save ten frames before recording'''
    frame_storage_A = deque(maxlen = frame_interval)
    frame_storage_B = deque(maxlen = frame_interval)
    frame_storage_color_A = deque(maxlen = frame_interval)
    frame_storage_color_B = deque(maxlen = frame_interval)
    timestamp_storage_A = deque(maxlen = frame_interval)
    timestamp_storage_B = deque(maxlen = frame_interval)


    '''Initilization'''
    previous_frame = None
    recording = False

    previous_frame_B = None
    recording_B = False

    difference = None
    difference_B = None

    # number of frames between background updates
    counter_A = 0
    counter_B = 0

    # timing for stopping
    start_time = None
    is_timing = False
    
    start_time_B = None
    is_timing_B = False
    
    FPS = 1/timeFPS

    num_vedio = 0   # naming the videos for camera A
    num_vedio_B = 0 # naming the videos for camera B
    
    writer_A = None
    writer_B = None
    
    '''Initialize camera'''
    cam0_enabled = False
    cam1_enabled = False

    try:
        picam2_A = Picamera2(0)
        video_config_A = picam2_A.create_still_configuration(main={"size": img_Size, "format": "RGB888"})
        picam2_A.configure(video_config_A)
        picam2_A.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 2.0})
        picam2_A.start()
        print("cam0 started", flush=True)
        cam0_enabled = True
    except Exception as e:
        print("cam0 failed", flush=True)
        picam2_A = None

    try:
        picam2_B = Picamera2(1)
        video_config_B = picam2_B.create_still_configuration(main={"size": img_Size, "format": "RGB888"})
        picam2_B.configure(video_config_B)
        picam2_B.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 2.0})
        picam2_B.start()
        print("cam1 started", flush=True)
        cam1_enabled = True
    except Exception as e:
        print("cam1 failed", flush=True)
        picam2_B = None

    

    try:
        while True:
            trialTime = time.time()
            # set motion detected as False at first
            motion_detected = False
            motion_detected_B = False

            # Capture current frame
            if cam0_enabled:
                frame = picam2_A.capture_array()
                sensor_timestamp_A = dt.datetime.now()

            if cam1_enabled:
                frame_B = picam2_B.capture_array()            
                sensor_timestamp_B = dt.datetime.now()

            if cam0_enabled:
                if frame.shape[2] != 3:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            if cam1_enabled:
                if frame_B.shape[2] != 3:
                    frame_B = cv2.cvtColor(frame_B, cv2.COLOR_BGRA2BGR)

            # Convert to grayscale, blur, and threshold
            if cam0_enabled:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gausBlur = cv2.GaussianBlur(gray, (5, 5), 0)

            if cam1_enabled:
                gray_B = cv2.cvtColor(frame_B, cv2.COLOR_BGR2GRAY)
                gausBlur_B = cv2.GaussianBlur(gray_B, (5, 5), 0)
            
            if cam0_enabled:
                frame_storage_A.append(gausBlur)
                frame_storage_color_A.append(frame)
                timestamp_storage_A.append(sensor_timestamp_A)

            if cam1_enabled:
                frame_storage_B.append(gausBlur_B)
                frame_storage_color_B.append(frame_B)
                timestamp_storage_B.append(sensor_timestamp_B)
            
            
            if (cam0_enabled and len(frame_storage_A) >= frame_interval) or (cam1_enabled and len(frame_storage_B) >= frame_interval):

                if cam0_enabled and len(frame_storage_A) >= frame_interval:
                    previous_frame = frame_storage_A[0]
                    counter_A += 1

                    # Compute absolute difference
                    difference = cv2.absdiff(gausBlur, previous_frame)

                    # Threshold
                    _, binaryImage = cv2.threshold(difference, 20, 255, cv2.THRESH_BINARY)

                    # Motion detection
                    white_A = cv2.countNonZero(binaryImage)
                    motion_detected = white_A > int(gray.size * motion_threshold)

                if cam1_enabled and len(frame_storage_B) >= frame_interval:
                    previous_frame_B = frame_storage_B[0]
                    counter_B += 1

                    # Compute absolute difference
                    difference_B = cv2.absdiff(gausBlur_B, previous_frame_B)

                    # Threshold
                    _, binaryImage_B = cv2.threshold(difference_B, 20, 255, cv2.THRESH_BINARY)

                    # Motion detection
                    white_B = cv2.countNonZero(binaryImage_B)
                    motion_detected_B = white_B > int(gray_B.size * motion_threshold)

                
                '''mark where is moving when enable_contour is true'''
                if cam0_enabled:
                    if motion_detected and enable_contour:
                        # 1) will use green rectangle to mark motion
                        dilated_A = cv2.dilate(binaryImage, None, iterations=2)
                        cnts_A, _ = cv2.findContours(dilated_A.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        
                        for c in cnts_A:
                            if cv2.contourArea(c) > 500:
                                if enable_preview:
                                    (x, y, w, h) = cv2.boundingRect(c)
                                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    print("Motion detected(A):", motion_detected, flush=True)

                        
                '''same function for camera B'''
                if cam1_enabled:
                    if motion_detected_B and enable_contour:    
                        # Camera B
                        dilated_B = cv2.dilate(binaryImage_B, None, iterations=2)
                        cnts_B, _ = cv2.findContours(dilated_B.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        for c_B in cnts_B:
                            if cv2.contourArea(c_B) > 500:
                                if enable_preview:
                                    (x_B, y_B, w_B, h_B) = cv2.boundingRect(c_B)
                                    cv2.rectangle(frame_B, (x_B, y_B), (x_B + w_B, y_B + h_B), (0, 255, 0), 2)
                    print("Motion detected(B):", motion_detected_B, flush=True)
                
                if cam0_enabled:
                    if recording and writer_A is not None:
                        writer_A.write(frame)
                        timestamp_data_A["frames"].append({
                            "frame_index": frame_index_A,
                            "timestamp": str(sensor_timestamp_A)
                        })
                        frame_index_A += 1

                    '''Start recording when motion detected'''
                    if motion_detected and (not recording):
                        frame_index_A = 0
                        print("Start recording... [Camera A]", flush=True)
                
                        num_vedio += 1
                        filename_A = f"{folder_path}/camera_A_{num_vedio}.mp4"
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        writer_A = cv2.VideoWriter(filename_A, fourcc, FPS, img_Size)

                        # create a json file to save frame
                        timestamp_data_A = {
                            "video_filename": filename_A,
                            "start_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),  
                            "frames": []
                        }
                        
                        # write previous 10 frames into vedio when motion detected
                        for each in frame_storage_color_A:
                            writer_A.write(each)
                            
                        start_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                        with open(log_file_path, "a") as f:
                            f.write(f"Video {num_vedio} (File: {folder_path}/camera_A_{num_vedio}.mp4): Start at {start_timestamp}\n")
                        
                        # write each frame into json file
                        for each_timestamp_A in timestamp_storage_A:
                            timestamp_data_A["frames"].append({
                                "frame_index": frame_index_A,
                                "timestamp": str(each_timestamp_A)  
                            })
                            frame_index_A += 1

                        recording = True

                        # cancel timing because new motion detected
                        if is_timing:
                            is_timing = False

                    # cancel timing because new motion detected
                    elif motion_detected and is_timing:
                        is_timing = False
                        start_time = None

                    # give 2 seconds buffer to stopr recording
                    elif (not motion_detected) and recording and (not is_timing):
                        start_time = time.time()
                        is_timing = True
                        print("No move detected (Camera A), will stop recording if no move detected in 2 seconds", flush=True)

                    # if no motion detected in 2 seconds, stop recording
                    elif is_timing and recording:
                        elapsed_time = time.time() - start_time
                        if elapsed_time > delay and recording:
                            stop_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                            with open(log_file_path, "a") as f:
                                f.write(f"Video {num_vedio} (File: {folder_path}/camera_A_{num_vedio}.mp4): Stop at {stop_timestamp}\n")

                            if timestamp_data_A is not None:
                                timestamp_data_A["stop_time"] = stop_timestamp
                                
                                timestamp_json_file = f"camera_A_{num_vedio}_timestamp.json"
                                timestamp_json_file_path = os.path.join(folder_path, timestamp_json_file)
                                
                                with open(timestamp_json_file_path, "w", encoding="utf-8") as jf:
                                    json.dump(timestamp_data_A, jf, indent=4)

                                timestamp_data_A = None

                            recording = False
                            is_timing = False
                            writer_A.release()
                            writer_A = None
                            print("Stop recording [Camera A]", flush=True)
                
                if cam1_enabled:
                    if recording_B and writer_B is not None:
                        writer_B.write(frame_B)
                        timestamp_data_B["frames"].append({
                            "frame_index": frame_index_B,
                            "timestamp": str(sensor_timestamp_B)
                        })
                        frame_index_B += 1
                    '''Same logic for camera B as A, have a look at A as reference'''
                    if motion_detected_B and (not recording_B):
                        frame_index_B = 0
                        print("Start recording... [Camera B]", flush=True)

                        num_vedio_B += 1
                        filename_B = f"{folder_path}/camera_B_{num_vedio_B}.mp4"
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        writer_B = cv2.VideoWriter(filename_B, fourcc, FPS, img_Size)

                        timestamp_data_B = {
                            "video_filename": filename_B,
                            "start_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),  
                            "frames": []
                        }
                        
                        # timestamp_file_B = f"camera_B_{num_vedio_B}_timestamp.txt"
                        # timestamp_file_path_B = os.path.join(folder_path, timestamp_file_B)
                        # if os.path.exists(timestamp_file_path_B):
                        #     open(log_file, "w").close()
                        
                        for each in frame_storage_color_B:
                            writer_B.write(each)
                            
                        start_timestamp_B = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                        with open(log_file_path_B, "a") as f:
                            f.write(f"Video {num_vedio_B} (File: {folder_path}/camera_B_{num_vedio_B}.h264): Start at {start_timestamp_B}\n")
                        # with open(timestamp_file_path_B, "a") as f:
                        #     for each_timestamp_B in timestamp_storage_B:
                        #         f.write(f"{frame_index_B}, {each_timestamp_B}\n")
                        #         frame_index_B += 1
                        for each_timestamp_B in timestamp_storage_B:
                            timestamp_data_B["frames"].append({
                                "frame_index": frame_index_B,
                                "timestamp": str(each_timestamp_B)  
                            })
                            frame_index_B += 1  

                        recording_B = True
                        if is_timing_B:
                            is_timing_B = False
                            
                    elif motion_detected_B and is_timing_B:
                        is_timing_B = False
                        start_time_B = None
                    elif (not motion_detected_B) and recording_B and (not is_timing_B):
                        start_time_B = time.time()
                        is_timing_B = True
                        print("No move detected (Camera B), will stop recording if no move detected in 2 seconds", flush=True)
                    elif is_timing_B and recording_B:
                        elapsed_time_B = time.time() - start_time_B
                        if elapsed_time_B > delay and recording_B:
                            stop_timestamp_B = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                            with open(log_file_path_B, "a") as f:
                                f.write(f"Video {num_vedio_B} (File: {folder_path}/camera_B_{num_vedio_B}.h264): Stop at {stop_timestamp_B}\n")
                            
                            if timestamp_data_B is not None:
                                timestamp_data_B["stop_time"] = stop_timestamp_B
                                
                                timestamp_json_file = f"camera_B_{num_vedio_B}_timestamp.json"
                                timestamp_json_file_path = os.path.join(folder_path, timestamp_json_file)
                                
                                with open(timestamp_json_file_path, "w", encoding="utf-8") as jf:
                                    json.dump(timestamp_data_B, jf, indent=4)

                                timestamp_data_B = None
                            
                            recording_B = False
                            is_timing_B = False
                            writer_B.release()
                            writer_B=None
                            print("Stop recording [Camera B]", flush=True)



            
                
            # preview function
            if enable_preview:
                preview_list = []

                if cam0_enabled:
                    if difference is not None:
                        color_diff_A = cv2.cvtColor(difference, cv2.COLOR_GRAY2BGR)
                    else:
                        color_diff_A = frame  
                    preview_A = np.hstack((color_diff_A, frame))
                    preview_list.append(preview_A)

                if cam1_enabled:
                    if difference_B is not None:
                        color_diff_B = cv2.cvtColor(difference_B, cv2.COLOR_GRAY2BGR)
                    else:
                        color_diff_B = frame_B  
                    preview_B = np.hstack((color_diff_B, frame_B))
                    preview_list.append(preview_B)

                if preview_list:
                    preview = np.vstack(preview_list)
                    preview_resized = cv2.resize(preview, (0, 0), fx=0.2, fy=0.2)
                    cv2.imshow("Camera_preview(A and B)", preview_resized)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                endTime = time.time()
                if endTime - trialTime < timeFPS:
                    time.sleep(timeFPS - (endTime - trialTime))

    finally:
        if writer_A is not None:
            writer_A.release()
        if writer_B is not None:
            writer_B.release()
        if cam0_enabled:
            picam2_A.stop()
        if cam1_enabled:
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

    # image size
    parser.add_argument("--resolution", default="4608x2592",
                        help="Image resolution WxH, e.g. 2304x1296")
    # number of frames stored in queue
    parser.add_argument("--frame_interval", type=int, default=10,
                        help="Frames between background updates")
    # Seconds per frame
    parser.add_argument("--timeFPS", type=float, default=1.2,
                        help="Seconds per frame (inverse of FPS)")

    parser.add_argument("--delay", type=float, default=2.0,
                    help="Time delay after there is no motion detected")
    parser.add_argument("--motion_threshold", type=float, default=0.005,
                help="threshold to detect whether there is motion detection")
    args = parser.parse_args()

    main(enable_preview=args.enable_preview, enable_contour=args.enable_contour, resolution=args.resolution, timeFPS=args.timeFPS, frame_interval=args.frame_interval, 
         delay=args.delay, motion_threshold=args.motion_threshold)

