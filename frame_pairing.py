import json
from datetime import datetime
import numpy as np

def sort(timestamp, timestamp_list, left=0, right=None):
    if right is None:
        right = len(timestamp_list)
    
    if left >= right:
        return left

    mid = (left + right) // 2
    if timestamp < timestamp_list[mid]:
        return sort(timestamp, timestamp_list, left, mid)
    else:
        return sort(timestamp, timestamp_list, mid + 1, right)

def frame_pairing(A_path, B_path, FPS = 1):
    # transfer json file format to python dictionary
    with open (A_path, 'r', encoding='UTF-8') as f:
        data_A = json.load(f)

    with open (B_path, 'r', encoding='UTF-8') as f:
        data_B = json.load(f)

    # get "frames" from dictionary, if no content, output an empty list
    timestamps_A = []
    frames_A = data_A.get("frames", [])

    timestamps_B = []
    frames_B = data_B.get("frames", [])
    
    threshold = 2/FPS

    for frame in frames_A:
        timestamp_A = frame.get("timestamp")
        ts_A = datetime.strptime(timestamp_A, "%Y-%m-%d %H:%M:%S.%f")
        timestamps_A.append(ts_A)

    for frame in frames_B:
        timestamp_B = frame.get("timestamp")
        ts_B = datetime.strptime(timestamp_B, "%Y-%m-%d %H:%M:%S.%f")
        timestamps_B.append(ts_B)
        
    delta = timestamps_A[23] - timestamps_A[0]
    pair = []
    for each_A in timestamps_A:
        index = sort(each_A, timestamps_B)
        closest_distance = np.inf
        closest_timestamp = None
        if index == 0:
            closest_timestamp = timestamps_B[0]
            closest_distance = abs(each_A - closest_timestamp).total_seconds()
        elif index >= len(timestamps_B):
            closest_timestamp = timestamps_B[len(timestamps_B)-1]
            closest_distance = abs(each_A - closest_timestamp).total_seconds()
        else:
            for each_B in [timestamps_B[index-1], timestamps_B[index]]:
                distance = abs((each_A - each_B).total_seconds())
                if distance < closest_distance:
                    closest_distance = distance
                    closest_timestamp = each_B
        if closest_distance < threshold:
            pair.append((
                frames_A[timestamps_A.index(each_A)]["frame_index"],
                frames_B[timestamps_B.index(closest_timestamp)]["frame_index"],
                each_A,
                closest_timestamp
            ))

    output_json = []

    for item in pair:
        frame_index_A, frame_index_B, timestamp_A, timestamp_B = item
        output_json.append({
            "frame_index_A": frame_index_A,
            "frame_index_B": frame_index_B,
            "timestamp_A": timestamp_A.strftime("%Y-%m-%d %H:%M:%S.%f"),
            "timestamp_B": timestamp_B.strftime("%Y-%m-%d %H:%M:%S.%f"),
            "time_difference": abs((timestamp_A - timestamp_B).total_seconds())
        })

    with open("paired_frames.json", "w", encoding="utf-8") as f:
        json.dump(output_json, f, indent=4)

    return pair

if __name__ == "__main__":
    path_A = "/home/terradynamics/Desktop/motion/2025-03-25_09-59_test/camera_A_2_timestamp.json"
    path_B = "/home/terradynamics/Desktop/motion/2025-03-25_09-59_test/camera_B_1_timestamp.json"
    pair = frame_pairing(path_A, path_B)