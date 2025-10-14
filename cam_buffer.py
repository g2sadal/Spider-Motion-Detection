import time
import datetime as dt
import argparse
import cv2
import numpy as np
import os
from picamera2 import Picamera2
from collections import deque
import threading
import json
from queue import Queue, Empty
from libcamera import controls

# Thread-safe queues
frame_queue_A = Queue(maxsize=50)
frame_queue_B = Queue(maxsize=50)

# Initialize cameras globally
picam2_A = None
picam2_B = None

# Parse resolution
try:
    w, h = map(int, args.resolution.lower().split('x'))
    img_Size = (w, h)
except ValueError:
    raise ValueError("--resolution must be 'widthxheight', e.g. 2304x1296")

buffLen = args.frame_interval
thresh_val = args.motion_threshold
FPS = 1 / args.timeFPS  # Convert seconds per frame to FPS
focus_distance = args.focus_distance

# Calculate lens position from focus distance
if focus_distance >= 10:
    lens_position = 0.0
else:
    lens_position = 1.0 / focus_distance
    lens_position = min(10.0, max(0.0, lens_position))

print(f"Focus Distance: {focus_distance}m -> Lens Position: {lens_position:.2f}", flush=True)

#Stop events
stop_event_A = threading.Event()
stop_event_B = threading.Event()

def cameraBuffer(camera_id, frame_queue, stop_event, camName):
    """Thread 1: Continuous frame capture"""
    try:
        # Initialize camera
        camera = Picamera2(camera_id)
        video_config = camera.create_still_configuration(
            main={"size": img_Size, "format": "RGB888"}
        )
        camera.configure(video_config)
        
        # Set focus
        camera.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": lens_position
        })
        
        camera.start()
        print(f"[{camName}] Camera started", flush=True)
        
        frame_count = 0
        
        # Run continuously until stop event
        while not stop_event.is_set():
            try:
                # Capture frame
                frame = camera.capture_array()
                sensor_timestamp = dt.datetime.now()
                
                # Convert color if needed
                if frame.shape[2] != 3:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                
                # Put in queue (thread-safe)
                try:
                    frame_queue.put((frame.copy(), sensor_timestamp), timeout=1.0)
                    frame_count += 1
                    
                    if frame_count % 50 == 0:
                        print(f"[{camName}] Captured {frame_count} frames", flush=True)
                        
                except:
                    pass  # Queue full, skip frame
                    
            except Exception as e:
                if not stop_event.is_set():
                    print(f"[{camName}] Capture error: {e}", flush=True)
                time.sleep(0.1)
        
        camera.stop()
        print(f"[{camName}] Camera stopped", flush=True)
        
    except Exception as e:
        print(f"[{camName}] Camera init failed: {e}", flush=True)
        

def motionDetect(frame_queue, stop_event, camName, folder_path, log_file_path, delay):
    """Thread 2: Motion detection and recording"""
    # Initialize all variables
    recording = False
    is_timing = False
    num_video = 0
    writer = None
    timestamp_data = None
    frame_index = 0
    start_time = None
    
    # Buffers
    gray_buffer = deque(maxlen=buffLen)
    color_buffer = deque(maxlen=buffLen)
    timestamp_buffer = deque(maxlen=buffLen)
    
    print(f"[{camName}] Motion detection started", flush=True)
    
    try:
        while not stop_event.is_set():
            try:
                # Get from Queue with timeout
                color_frame, timestamp = frame_queue.get(timeout=1.0)
                
                # Process for motion detection
                gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
                gaus_blur = cv2.GaussianBlur(gray, (5, 5), 0)
                
                # Store BOTH color and gray
                color_buffer.append(color_frame.copy())
                gray_buffer.append(gaus_blur)
                timestamp_buffer.append(timestamp)
                
                # Need enough frames
                if len(gray_buffer) < buffLen:
                    continue
                
                # Compare adjacent frames
                prev_gray = gray_buffer[0]
                curr_gray = gaus_blur
                
                # Motion detection
                diff = cv2.absdiff(prev_gray, curr_gray)
                _, binary_image = cv2.threshold(diff, 20, 255, cv2.THRESH_BINARY)
                white = cv2.countNonZero(binary_image)
                motion_detected = white > int(curr_gray.size * thresh_val)
                
                # RECORDING LOGIC
                if motion_detected:
                    if recording and writer is not None:
                        # Continue recording
                        writer.write(color_frame)
                        timestamp_data["frames"].append({
                            "frame_index": frame_index,
                            "timestamp": str(timestamp)
                        })
                        frame_index += 1
                    else:
                        # START RECORDING
                        frame_index = 0
                        num_video += 1
                        
                        filename = f"{folder_path}/{camName}_{num_video}.mp4"
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        writer = cv2.VideoWriter(filename, fourcc, FPS, img_Size)
                        
                        if not writer.isOpened():
                            print(f"[{camName}] ERROR: Could not open video writer", flush=True)
                            writer = None
                            continue
                        
                        timestamp_data = {
                            "video_filename": filename,
                            "start_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                            "focus_distance": focus_distance,
                            "lens_position": lens_position,
                            "frames": []
                        }
                        
                        # Write buffered frames
                        buffered_colors = list(color_buffer)
                        buffered_timestamps = list(timestamp_buffer)
                        
                        for buf_frame, buf_ts in zip(buffered_colors, buffered_timestamps):
                            writer.write(buf_frame)
                            timestamp_data["frames"].append({
                                "frame_index": frame_index,
                                "timestamp": str(buf_ts)
                            })
                            frame_index += 1
                        
                        # Log start
                        start_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                        with open(log_file_path, "a") as f:
                            f.write(f"Video {num_video} ({camName}): Start at {start_timestamp}\n")
                        
                        recording = True
                        is_timing = False
                        print(f"[{camName}] Started recording video {num_video}", flush=True)
                
                else:
                    # NO MOTION
                    if recording:
                        if is_timing:
                            elapsed_time = time.time() - start_time
                            
                            if elapsed_time > delay:
                                # STOP RECORDING
                                stop_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                                
                                with open(log_file_path, "a") as f:
                                    f.write(f"Video {num_video} ({camName}): Stop at {stop_timestamp}\n")
                                
                                if timestamp_data is not None:
                                    timestamp_data["stop_time"] = stop_timestamp
                                    
                                    json_file = f"{camName}_{num_video}_timestamp.json"
                                    json_path = os.path.join(folder_path, json_file)
                                    
                                    with open(json_path, "w", encoding="utf-8") as jf:
                                        json.dump(timestamp_data, jf, indent=4)
                                
                                if writer is not None:
                                    writer.release()
                                    writer = None
                                
                                recording = False
                                is_timing = False
                                print(f"[{camName}] Stopped recording: {frame_index} frames", flush=True)
                            else:
                                # Continue recording
                                writer.write(color_frame)
                                timestamp_data["frames"].append({
                                    "frame_index": frame_index,
                                    "timestamp": str(timestamp)
                                })
                                frame_index += 1
                        else:
                            # Start timing
                            start_time = time.time()
                            is_timing = True
                            writer.write(color_frame)
                            timestamp_data["frames"].append({
                                "frame_index": frame_index,
                                "timestamp": str(timestamp)
                            })
                            frame_index += 1
                            print(f"[{camName}] No motion, will stop in 2s", flush=True)
                
            except Empty:
                continue
            except Exception as e:
                if not stop_event.is_set():
                    print(f"[{camName}] Detection error: {e}", flush=True)
                time.sleep(0.1)
        
        # Cleanup
        if recording and writer is not None:
            writer.release()
        print(f"[{camName}] Motion detection stopped", flush=True)
        
    except Exception as e:
        print(f"[{camName}] Fatal error: {e}", flush=True)
        import traceback
        traceback.print_exc()    
        
        
if __name__ == "__main__":
    # Add argparse
    parser = argparse.ArgumentParser(description="Parallel dual camera motion detection")
    
    parser.add_argument("--resolution", default="4608x2592",
                        help="Image resolution WxH, e.g. 2304x1296")
    parser.add_argument("--frame_interval", type=int, default=10,
                        help="Frames between background updates")
    parser.add_argument("--timeFPS", type=float, default=1.2,
                        help="Seconds per frame (inverse of FPS)")
    parser.add_argument("--delay", type=float, default=2.0,
                        help="Time delay after no motion detected")
    parser.add_argument("--motion_threshold", type=float, default=0.005,
                        help="Threshold for motion detection")
    parser.add_argument("--focus_distance", type=float, default=0.5,
                        help="Focus distance in meters (0.1-10)")
    parser.add_argument("--folder_name", type=str, default=None,
                        help="Folder name to use for saving files")
    
    args = parser.parse_args()
    
    # Parse resolution
    try:
        w, h = map(int, args.resolution.lower().split('x'))
        img_Size = (w, h)
    except ValueError:
        raise ValueError("--resolution must be 'widthxheight', e.g. 2304x1296")

    buffLen = args.frame_interval
    thresh_val = args.motion_threshold
    FPS = 1 / args.timeFPS
    focus_distance = args.focus_distance

    # Calculate lens position from focus distance
    if focus_distance >= 10:
        lens_position = 0.0
    else:
        lens_position = 1.0 / focus_distance
        lens_position = min(10.0, max(0.0, lens_position))

    print(f"Focus Distance: {focus_distance}m -> Lens Position: {lens_position:.2f}", flush=True)
    
    # Use the folder name from UI, or generate one if not provided
    base_dir = os.path.dirname(os.path.abspath(__file__))
    if args.folder_name:
        folder_name = args.folder_name
    else:
        folder_name = dt.datetime.now().strftime("%Y-%m-%d_%H-%M_test")
    folder_path = os.path.join(base_dir, folder_name)
    os.makedirs(folder_path, exist_ok=True)
    
    '''Create a txt file to save the starting time and ending time of each vedio'''
    log_file_A = "motion_timestamps_A.txt"
    log_file_path_A = os.path.join(folder_path, log_file_A)
    open(log_file_path_A, "w").close()
    
    log_file_B = 'motion_timestamps_B.txt'
    log_file_path_B = os.path.join(folder_path, log_file_B)
    open(log_file_path_B, 'w').close()

    print("="*60)
    print("Starting Parallel Motion Detection System")
    print(f"Output folder: {folder_path}")
    print("="*60)
        
    # Create threads
    t1 = threading.Thread(target = cameraBuffer, args = (0, frame_queue_A, stop_event_A, "cam_A"), daemon=True)
    t2 = threading.Thread(target = cameraBuffer, args = (1, frame_queue_B, stop_event_B, "cam_B"), daemon=True)
    t3 = threading.Thread(target = motionDetect, args = (frame_queue_A, stop_event_A, "cam_A", folder_path, log_file_path_A, args.delay), daemon=True)
    t4 = threading.Thread(target = motionDetect, args = (frame_queue_B, stop_event_B, "cam_B", folder_path, log_file_path_B, args.delay), daemon=True)
    
    # Start all threads
    t1.start()
    t2.start()
    t3.start()
    t4.start()

    print("\nSystem running. Press Ctrl+C to stop.\n")
    
    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\nStopping system...")
        
        # Signal all threads to stop
        stop_event_A.set()
        stop_event_B.set()
        
        # Wait for threads
        t1.join(timeout=5)
        t2.join(timeout=5)
        t3.join(timeout=5)
        t4.join(timeout=5)
        
        print("="*60)
        print("System stopped")
        print("="*60)
        
    #finally:
   #     if writer_A is not None:
  #          writer_A.release()
 #       if writer_B is not None:
#            writer_B.release()

