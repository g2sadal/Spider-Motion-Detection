import cv2
import json
import datetime

import cv2
import json

def find_frame(
    video_path: str, 
    json_path: str, 
    target: int
):
    
    target_time = datetime.datetime.fromisoformat(target)

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    
    frames_list = data["frames"]

    closest = None
    min_diff = None
    
    for item in frames_list:
        current = item["frame_index"]
        timestamp = item["timestamp"]
        time_current = datetime.datetime.fromisoformat(timestamp)
        diff = abs(time_current - target_time)
        if min_diff is None or diff < min_diff:
            min_diff = diff
            closest = current

    print(f"frame finded: {closest}")
    print(f"difference: {min_diff}")
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, closest)
    ret, frame = cap.read()
    cap.release()
    return frame

if __name__ == "__main__":
    video_path = "/home/terradynamics/Desktop/motion/2025-03-25_09-59_test/camera_B_1.mp4"
    json_path  = "/home/terradynamics/Desktop/motion/2025-03-25_09-59_test/camera_B_1_timestamp.json"
    target = "2025-03-25 10:00:48.485214"

    found_frame = find_frame(video_path, json_path, target)
    if found_frame is not None:
        height, width = found_frame.shape[:2]
        scale = 0.2
        resized = cv2.resize(found_frame, (int(width*scale), int(height*scale)))
        cv2.imshow("Closest Frame", resized)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
