import time
import datetime as dt
import argparse
import cv2
import numpy as np
import os
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from libcamera import controls
from datetime import datetime
from collections import deque
import json

def add_timestamp_overlay(frame, timestamp, camera_name=""):
    """
    Add timestamp overlay to frame
    """
    # Create a copy to avoid modifying original
    frame_copy = frame.copy()
    
    # Format timestamp string
    timestamp_str = timestamp.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    if camera_name:
        display_text = f"{camera_name}: {timestamp_str}"
    else:
        display_text = timestamp_str
    
    # Text properties
    font = cv2.FONT_HERSHEY_SIMPLEX

    font_scale = 1.2
    color = (255, 255, 255)  # White text
    thickness = 3

    background_color = (0, 0, 0)  # Black background
    
    # Get text size for background rectangle
    (text_width, text_height), baseline = cv2.getTextSize(display_text, font, font_scale, thickness)
    
    # Position (top-left corner with margin)
    x, y = 10, 30
    
    # Draw black background rectangle
    cv2.rectangle(frame_copy, (x-5, y-text_height-5), (x+text_width+5, y+baseline+5), background_color, -1)
    
    # Draw white text
    cv2.putText(frame_copy, display_text, (x, y), font, font_scale, color, thickness)
    
    return frame_copy

def calculate_lens_position(focus_distance_meters):
    """
    Convert focus distance in meters to lens position value.
    According to Picamera2 manual section 5.2.3:
    - LensPosition ranges from 0.0 (infinity) to ~10.0 (very close/macro)
    - The relationship is approximately: LensPosition = 1/distance_in_meters
    
    Args:
        focus_distance_meters: Distance in meters (0.1 to 10)
            - 0.1m = very close focus (macro)
            - 0.5m = close focus (for 70cm box)
            - 2.0m = medium distance
            - 10m = far/infinity focus

    Returns:
        lens_position: Value for LensPosition control (0.0 to 10.0)
    """
    # Clamp input to valid range
    focus_distance_meters = max(0.1, min(10.0, focus_distance_meters))

    if focus_distance_meters >= 10:
        # Far focus / infinity
        return 0.0
    else:
        # Use 1/distance relationship as per Picamera2 documentation
        # This gives: 0.1m->10, 0.5m->2, 1m->1, 2m->0.5, 10m->0.1
        lens_position = 1.0 / focus_distance_meters
        # Clamp to valid range
        return min(10.0, max(0.0, lens_position))

def main(enable_preview=False, enable_contour=False, frame_interval=10, resolution="4608x2592", 
         timeFPS=1.2, delay=2, motion_threshold=0.005, focus_distance=2.0, folder_name=None):
    try:
        w, h = map(int, resolution.lower().split('x'))
        img_Size = (w, h)
    except ValueError:
        raise ValueError("--resolution has to be 'width x height', such as 2304x1296")
    
    # Validate and calculate lens position from focus distance
    if focus_distance < 0.1 or focus_distance > 10:
        print(f"Warning: focus_distance {focus_distance}m outside range (0.1-10m), clamping", flush=True)
        focus_distance = max(0.1, min(10.0, focus_distance))
    
    lens_position = calculate_lens_position(focus_distance)
    print(f"Focus Distance: {focus_distance}m -> Lens Position: {lens_position:.2f}", flush=True)
    
    '''Create Json file to save the timestamp of each frame of video A and B'''
    timestamp_data_A = None
    timestamp_data_B = None


    base_dir = os.path.dirname(os.path.abspath(__file__))
    if folder_name:  # Use provided folder name if given
        folder_name = folder_name
    else:  # Otherwise generate one
        folder_name = datetime.now().strftime("%Y-%m-%d_%H-%M_test")
    folder_path = os.path.join(base_dir, folder_name)
    os.makedirs(folder_path, exist_ok=True)
    
    '''Create a txt file to save the starting time and ending time of each video'''
    log_file = "motion_timestamps.txt"
    log_file_path = os.path.join(folder_path, log_file)
    
    if os.path.exists(log_file_path):
        open(log_file_path, "w").close()

    log_file_B = 'motion_timestamps_B.txt'
    log_file_path_B = os.path.join(folder_path, log_file_B)

    if os.path.exists(log_file_path_B):
        open(log_file_path_B, 'w').close()
        
    '''frame index initialization(for frame alignment)'''
    frame_index_A = 0
    frame_index_B = 0

    '''Create a queue to save frames before recording'''
    frame_storage_A = deque(maxlen = frame_interval)
    frame_storage_B = deque(maxlen = frame_interval)
    frame_storage_color_A = deque(maxlen = frame_interval)
    frame_storage_color_B = deque(maxlen = frame_interval)
    timestamp_storage_A = deque(maxlen = frame_interval)
    timestamp_storage_B = deque(maxlen = frame_interval)

    '''Initialization'''
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

    num_video = 0   # naming the videos for camera A
    num_video_B = 0 # naming the videos for camera B
    
    writer_A = None
    writer_B = None
    
    '''Initialize camera with adjustable focus'''
    cam0_enabled = False
    cam1_enabled = False

    try:
        picam2_A = Picamera2(0)
        video_config_A = picam2_A.create_still_configuration(main={"size": img_Size, "format": "RGB888"})
        picam2_A.configure(video_config_A)

        # Apply calculated focus position
        picam2_A.set_controls({
            "AfMode": controls.AfModeEnum.Manual,  # Manual focus mode
            "LensPosition": lens_position  # Use calculated lens position
        })

        picam2_A.start()
        print(f"cam0 started with focus at {focus_distance}m (lens position: {lens_position:.2f})", flush=True)
        cam0_enabled = True
    except Exception as e:
        print(f"cam0 failed: {e}", flush=True)
        picam2_A = None

    try:
        picam2_B = Picamera2(1)
        video_config_B = picam2_B.create_still_configuration(main={"size": img_Size, "format": "RGB888"})
        picam2_B.configure(video_config_B)

        # Apply same focus position to camera B
        picam2_B.set_controls({
            "AfMode": controls.AfModeEnum.Manual,  # Manual focus mode
            "LensPosition": lens_position  # Use calculated lens position
        })

        picam2_B.start()
        print(f"cam1 started with focus at {focus_distance}m (lens position: {lens_position:.2f})", flush=True)
        cam1_enabled = True
    except Exception as e:
        print(f"cam1 failed: {e}", flush=True)
        picam2_B = None

    try:
        while True:
            trialTime = time.time()
            # set motion detected as False at first
            motion_detected = False
            motion_detected_B = False
            contour = False
            contour_B = False

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

                    dilated_A = cv2.dilate(binaryImage, None, iterations=2)
                    cnts_A, _ = cv2.findContours(dilated_A.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        
                    for c in cnts_A:
                        if cv2.contourArea(c) > 100:
                            contour = True
                            break
                            
                    #motion_detected = (white_A > 20) and contour
                    motion_detected = (white_A / (img_Size[0] * img_Size[1]) > motion_threshold) and contour

                if cam1_enabled and len(frame_storage_B) >= frame_interval:
                    previous_frame_B = frame_storage_B[0]
                    counter_B += 1

                    # Compute absolute difference
                    difference_B = cv2.absdiff(gausBlur_B, previous_frame_B)

                    # Threshold
                    _, binaryImage_B = cv2.threshold(difference_B, 20, 255, cv2.THRESH_BINARY)

                    # Motion detection
                    white_B = cv2.countNonZero(binaryImage_B)

                    dilated_B = cv2.dilate(binaryImage_B, None, iterations=2)
                    cnts_B, _ = cv2.findContours(dilated_B.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for c_B in cnts_B:
                        if cv2.contourArea(c_B) > 500:
                            contour_B = True
                            break
                            
                    #motion_detected_B = (white_B > 20) and contour_B
                    motion_detected_B = (white_B / (img_Size[0] * img_Size[1]) > motion_threshold) and contour_B

                '''mark where is moving when enable_contour is true'''
                if cam0_enabled:
                    if motion_detected and enable_contour:
                        # will use green rectangle to mark motion
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
                        timestamped_frame = add_timestamp_overlay(frame, sensor_timestamp_A, "Cam A")
                        writer_A.write(timestamped_frame)
                        timestamp_data_A["frames"].append({
                            "frame_index": frame_index_A,
                            "timestamp": str(sensor_timestamp_A)
                        })
                        frame_index_A += 1

                    '''Start recording when motion detected'''
                    if motion_detected and (not recording):
                        frame_index_A = 0
                        print("Start recording... [Camera A]", flush=True)
                
                        num_video += 1
                        filename_A = f"{folder_path}/camera_A_{num_video}.mp4"
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        writer_A = cv2.VideoWriter(filename_A, fourcc, FPS, img_Size)

                        # create a json file to save frame metadata
                        timestamp_data_A = {
                            "video_filename": filename_A,
                            "start_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                            "focus_distance": focus_distance,  # Save focus setting
                            "lens_position": lens_position,
                            "frames": []
                        }
                        
                        # write previous frames into video when motion detected
                        buffered_frames = list(frame_storage_color_A)
                        buffered_timestamps = list(timestamp_storage_A)
                        for frame_buf, timestamp_buf in zip(buffered_frames, buffered_timestamps):
                            timestamped_frame = add_timestamp_overlay(frame_buf, timestamp_buf, "CAM_A")
                            writer_A.write(timestamped_frame)
                            
                        start_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                        with open(log_file_path, "a") as f:
                            f.write(f"Video {num_video} (File: {folder_path}/camera_A_{num_video}.mp4): Start at {start_timestamp}\n")
                        
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

                    # give delay buffer to stop recording
                    elif (not motion_detected) and recording and (not is_timing):
                        start_time = time.time()
                        is_timing = True
                        print(f"No move detected (Camera A), will stop recording if no move detected in {delay} seconds", flush=True)

                    # if no motion detected after delay, stop recording
                    elif is_timing and recording:
                        elapsed_time = time.time() - start_time
                        if elapsed_time > delay and recording:
                            stop_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                            with open(log_file_path, "a") as f:
                                f.write(f"Video {num_video} (File: {folder_path}/camera_A_{num_video}.mp4): Stop at {stop_timestamp}\n")

                            if timestamp_data_A is not None:
                                timestamp_data_A["stop_time"] = stop_timestamp
                                
                                timestamp_json_file = f"camera_A_{num_video}_timestamp.json"
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
                        timestamped_frame_B = add_timestamp_overlay(frame_B, sensor_timestamp_B, "Cam B")   
                        writer_B.write(timestamped_frame_B)
                        timestamp_data_B["frames"].append({
                            "frame_index": frame_index_B,
                            "timestamp": str(sensor_timestamp_B)
                        })
                        frame_index_B += 1
                     
                    '''Same logic for camera B as A'''
                    if motion_detected_B and (not recording_B):
                        frame_index_B = 0
                        print("Start recording... [Camera B]", flush=True)

                        num_video_B += 1
                        filename_B = f"{folder_path}/camera_B_{num_video_B}.mp4"
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        writer_B = cv2.VideoWriter(filename_B, fourcc, FPS, img_Size)

                        timestamp_data_B = {
                            "video_filename": filename_B,
                            "start_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                            "focus_distance": focus_distance,  # Save focus setting
                            "lens_position": lens_position,
                            "frames": []
                        }
                        
                        buffered_frames_B = list(frame_storage_color_B)
                        buffered_timestamps_B = list(timestamp_storage_B)
                        for frame_buf_B, timestamp_buf_B in zip(buffered_frames_B, buffered_timestamps_B):
                            timestamped_frame_B = add_timestamp_overlay(frame_buf_B, timestamp_buf_B, "CAM_B")
                            writer_B.write(timestamped_frame_B)
                            
                        start_timestamp_B = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                        with open(log_file_path_B, "a") as f:
                            f.write(f"Video {num_video_B} (File: {folder_path}/camera_B_{num_video_B}.mp4): Start at {start_timestamp_B}\n")

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
                        print(f"No move detected (Camera B), will stop recording if no move detected in {delay} seconds", flush=True)

                    elif is_timing_B and recording_B:
                        elapsed_time_B = time.time() - start_time_B
                        if elapsed_time_B > delay and recording_B:
                            stop_timestamp_B = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                            with open(log_file_path_B, "a") as f:
                                f.write(f"Video {num_video_B} (File: {folder_path}/camera_B_{num_video_B}.mp4): Stop at {stop_timestamp_B}\n")

                            if timestamp_data_B is not None:
                                timestamp_data_B["stop_time"] = stop_timestamp_B
                                
                                timestamp_json_file = f"camera_B_{num_video_B}_timestamp.json"
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
            actual_frame_time = endTime - trialTime
            actual_fps = 1/actual_frame_time if actual_frame_time > 0 else 0
            print(f"[BENCHMARK] Frame time: {actual_frame_time:.4f}s | Actual FPS: {actual_fps:.2f}", flush=True)
            # Commented out to run at max speed:
            # if endTime - trialTime < timeFPS:
            #     time.sleep(timeFPS - (endTime - trialTime))
            

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
        description="Dual camera motion detection with adjustable focus control"
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
    # Delay after no motion
    parser.add_argument("--delay", type=float, default=2.0,
                        help="Time delay after there is no motion detected")
    # Motion threshold
    parser.add_argument("--motion_threshold", type=float, default=0.005,
                        help="Threshold to detect whether there is motion detection")
    # Focus distance parameter
    parser.add_argument("--focus_distance", type=float, default=2.0,
                        help="Focus distance in meters (0.1-10, where 0.1=close macro, 0.5=70cm box, 2=medium, 10=far)")
    
    parser.add_argument("--folder_name", type=str, default=None,
                        help="Folder name to use for saving files")
    args = parser.parse_args()

    main(enable_preview=args.enable_preview, 
         enable_contour=args.enable_contour, 
         resolution=args.resolution, 
         timeFPS=args.timeFPS, 
         frame_interval=args.frame_interval, 
         delay=args.delay, 
         motion_threshold=args.motion_threshold,
         focus_distance=args.focus_distance,
         folder_name=args.folder_name)

