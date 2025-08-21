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

    # frame index initialization(for frame allignment)
frame_index_A = 0
frame_index_B = 0

# Initialize camera
picam2_A = Picamera2(0)
picam2_B = Picamera2(1)

img_Size = (4608,2592) # max is 4608x2592, original is 2304x1296, ration is ~2:1
buffLen = 10 # number of frames to store in buffer
thresh_val = 0.005 # percentage of different pixels to qualify as motion
FPS = 1

video_config_A = picam2_A.create_still_configuration(main={"size": img_Size, "format": "RGB888"})
video_config_B = picam2_B.create_still_configuration(main={"size": img_Size, "format": "RGB888"})

picam2_A.configure(video_config_A)
picam2_B.configure(video_config_B)

frame_storage_color_A = []
frame_storage_color_B = []
timestamp_storage_A = []
timestamp_storage_B = []

buffLockA = threading.Lock()
buffLockB = threading.Lock()
motionLockA = threading.Lock()
motionLockB = threading.Lock()
buffEndA = threading.Event()
buffEndB = threading.Event()
motionEndA = threading.Event()
motionEndB = threading.Event()

def cameraBuffer(camera, frame_storage, time_storage, motionLock, buffLock, buffEnd, camName):
    motionLock.acquire()
    camera.start()

    #cv2.imshow(camName,np.zeros([1,1]))
    
    i = 0
    
    #while True and not buffEnd.is_set():
    for i in range(100):
        
        frame = camera.capture_array()
        sensor_timestamp = dt.datetime.now()
        
        if frame.shape[2] != 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        if i > buffLen:
            buffLock.acquire()
            print("BLA in B", camName)
        frame_storage.append(frame)   
        time_storage.append(sensor_timestamp)
        if i > buffLen:
            motionLock.release()
            print("MLR in B", camName)
        
        print(i, camName)
        i += 1
        
        #if cv2.waitKey(1) & 0xFF == ord('q'):
         #   buffEnd.set()
        
    

            
    #buffLock.acquire()
    camera.stop()
    buffEnd.set()
    motionLock.release()
        

def motionDetect(frame_storage, time_storage, motionLock, buffLock, buffEnd, camName):
    recording = False
    is_timing = False
    num_video = 0
    
    motionLock.acquire()
    size = len(frame_storage)
    buffLock.release()
    
    
    while buffLen+1 < size:
        # Get "current" frame and revious comparison frame
        print(camName, " size ", size)
        if not buffEnd.is_set():
            motionLock.acquire()
            print("MLA in M ", camName)
        currFrame = frame_storage[buffLen]
        currTimeStamp = time_storage[buffLen]
        prevFrame = frame_storage.pop(0)
        prevTimeStamp = time_storage.pop(0)
        if not buffEnd.is_set():
            buffLock.release()
            print("BLR in M ", camName)
        
        # Transform images for comparison
        grayCurr = cv2.cvtColor(currFrame, cv2.COLOR_BGR2GRAY)
        gausBlurCurr = cv2.GaussianBlur(grayCurr, (5, 5), 0)
        
        grayPrev = cv2.cvtColor(prevFrame, cv2.COLOR_BGR2GRAY)
        gausBlurPrev = cv2.GaussianBlur(grayPrev, (5, 5), 0)
        
        diff = cv2.absdiff(gausBlurPrev, gausBlurCurr)
        _, binaryImage = cv2.threshold(diff, 20, 255, cv2.THRESH_BINARY)
        white = cv2.countNonZero(binaryImage)
        motion_detected = white > int(grayCurr.size * thresh_val)
        
        
        if motion_detected:
            # If already recording, add frame and timestamp to files
            if recording:
                writer_A.write(currFrame)
                timestamp_data_A["frames"].append({
                    "frame_index": frame_index_A,
                    "timestamp": str(currTimeStamp)
                })
                frame_index_A += 1
            # If not already recording, open files and start recording
            else:
                # Open video and timestamp files
                frame_index_A = 0
                print("Start recording... ", camName)
         
                num_video += 1
                filename_A = f"{folder_path}/{camName}_{num_video}.mp4"
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer_A = cv2.VideoWriter(filename_A, fourcc, FPS, img_Size)

                # create a json file to save frame
                timestamp_data_A = {
                    "video_filename": filename_A,
                    "start_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),  
                    "frames": []
                }
                
                # Write 10 frames prior to frame to video
                if not buffEnd.is_set():
                    motionLock.acquire()
                    print("MLA in M ", camName)
                writer_A.write(prevFrame)
                timestamp_data_A["frames"].append({
                    "frame_index": frame_index_A,
                    "timestamp": str(prevTimeStamp)
                })
                frame_index_A = 1
                for i in range(buffLen-1):
                    frame = frame_storage.pop(0)
                    sensor_timestamp_A = time_storage.pop(0)
                    writer_A.write(frame)
                    timestamp_data_A["frames"].append({
                        "frame_index": frame_index_A,
                        "timestamp": str(sensor_timestamp_A)
                    })
                    frame_index_A += 1
                
                if not buffEnd.is_set():
                    buffLock.release()
                    print("BLR in M ", camName)
                    
                # Log start of recording to file
                start_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                with open(log_file_path, "a") as f:
                    f.write(f"Video {num_video} (File: {folder_path}/{camName}_{num_video}.h264): Start at {start_timestamp}\n")
                    
                # Turn recording on
                recording = True
        else:
            # If no motion and already recording
            if recording:
                # Check if timing since end of motion
                if is_timing:
                    elapsed_time = time.time() - start_time
                    
                    # If time elapsed is over threshold, stop recording and close all files
                    if elapsed_time > 2:
                        stop_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                        with open(log_file_path, "a") as f:
                            f.write(f"Video {num_video} (File: {folder_path}/{camName}_{num_video}.h264): Stop at {stop_timestamp}\n")

                        if timestamp_data_A is not None:
                            timestamp_data_A["stop_time"] = stop_timestamp
                            
                            timestamp_json_file = f"{camName}_{num_video}_timestamp.json"
                            timestamp_json_file_path = os.path.join(folder_path, timestamp_json_file)
                            
                            with open(timestamp_json_file_path, "w", encoding="utf-8") as jf:
                                json.dump(timestamp_data_A, jf, indent=4)

                            timestamp_data_A = None

                        recording = False
                        is_timing = False
                        writer_A.release()
                        writer_A = None
                        print("Stop recording ", camName)
                    # If time elapsed is not enough, continue recording
                    else:
                       writer_A.write(currFrame)
                       timestamp_data_A["frames"].append({
                           "frame_index": frame_index_A,
                           "timestamp": str(currTimeStamp)
                       })
                       frame_index_A += 1
                # If not timing yet, start timing and record to file
                else:
                    start_time = time.time()
                    
                    writer_A.write(currFrame)
                    timestamp_data_A["frames"].append({
                        "frame_index": frame_index_A,
                        "timestamp": str(currTimeStamp)
                    })
                    frame_index_A += 1
                       
                    is_timing = True
                    print("No move detected (",camName,"), will stop recording if no move detected in 2 seconds")
                    
        
    
        
        if not buffEnd.is_set():
            motionLock.acquire()
        size = len(frame_storage)
        if not buffEnd.is_set():
            buffLock.release()
        
        
    print("loop ended ", camName)    
        
        
if __name__ == "__main__":
    folder_name = dt.datetime.now().strftime("%Y-%m-%d_%H-%M_test")
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
        
    t1 = threading.Thread(target = cameraBuffer, args = (picam2_A, frame_storage_color_A, timestamp_storage_A, motionLockA, buffLockA, buffEndA, "cam A"), daemon = False)
    t2 = threading.Thread(target = cameraBuffer, args = (picam2_B, frame_storage_color_B, timestamp_storage_B, motionLockB, buffLockB, buffEndB, "cam B"), daemon = False)
    t3 = threading.Thread(target = motionDetect, args = (frame_storage_color_A, timestamp_storage_A, motionLockA, buffLockA, buffEndA, "cam A"), daemon = False)
    t4 = threading.Thread(target = motionDetect, args = (frame_storage_color_B, timestamp_storage_B, motionLockB, buffLockB, buffEndB, "cam B"), daemon = False)
    
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    
    print("threads joined")
    cv2.destroyAllWindows()
        
    #finally:
   #     if writer_A is not None:
  #          writer_A.release()
 #       if writer_B is not None:
#            writer_B.release()

