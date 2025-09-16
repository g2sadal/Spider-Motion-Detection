#!/usr/bin/env python3

import tkinter as tk
import subprocess, threading, queue, os, json
from datetime import datetime, timedelta
import pandas as pd
from tkinter import filedialog, messagebox
import time
import shutil
import glob

local_base_dir = None  
session_folder = ""
consolidated_json_folder = None  # Store path to consolidated JSON folder

DEFAULTS = {
    "resolution":        "4608x2592",
    "frame_interval":    "10",
    "timeFPS":           "1.2",
    "delay":             "2",
    "motion_threshold":  "0.005",
    "focus_distance":    "0.5" #default 0.5m for the setup environment
}

RES_CHOICES = [
    "4608x2592",
    "2304x1296",
    "1920x1080",
    "1280x720",
    "640x480"
]

q = queue.Queue()
procs = []
running = False

def print_to_box(msg):
    output_box.insert(tk.END, msg + "\n")
    output_box.see(tk.END)

def reader(name, stream):
    for line in iter(stream.readline, ''):
        q.put((name, line.rstrip()))
    stream.close()

def remote_capture_all():
    try:
        with open("IP.txt") as f:
            hosts = [line.strip().split() for line in f if len(line.strip().split()) == 2]
    except FileNotFoundError:
        messagebox.showerror("Error", "IP.txt not found")
        return

    remote_capture_dir = "/home/terradynamics/captures"
    capture_script = "/home/terradynamics/Desktop/motion/img_capture.py"

    for cam_name, ip in hosts:
        ssh_cmd = [
            "ssh", f"terradynamics@{ip}",
            f"mkdir -p {remote_capture_dir} && python3 {capture_script} --out_dir {remote_capture_dir}"
        ]

        if subprocess.call(ssh_cmd) == 0:
            print_to_box(f"[{cam_name}] capture done (saved on Pi)")
        else:
            print_to_box(f"[{cam_name}] capture failed")

def run_capture_thread():
    threading.Thread(target=remote_capture_all, daemon=True).start()

def build_remote_cmd():
    res   = res_var.get()
    fi    = fi_entry.get().strip()
    tpf   = fps_entry.get().strip()
    dly   = delay_entry.get().strip()
    thr   = thres_entry.get().strip()
    focus = focus_entry.get().strip()

    cmd = ["python3", "-u", "/home/terradynamics/Desktop/motion/dual_cam_v12.py",
           "--resolution", res]

    if fi:  cmd += ["--frame_interval", fi]
    if tpf: cmd += ["--timeFPS", tpf]
    if dly: cmd += ["--delay", dly]
    if thr: cmd += ["--motion_threshold", thr]
    if focus: cmd += ["--focus_distance", focus]

    return " ".join(cmd)

def launch_for_host(name, ip):
    remote_cmd = build_remote_cmd()
    try:
        proc = subprocess.Popen(
            ["ssh", f"terradynamics@{ip}", remote_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        def reader_thread(stream, stream_name):
            for line in iter(stream.readline, ''):
                if line:
                    q.put((name, f"[{stream_name}] {line.rstrip()}"))

                    if "cam0 started" in line:
                        set_pi_light(f"{name}_cam0", "green")
                    if "cam1 started" in line:
                        set_pi_light(f"{name}_cam1", "green")
                    if "cam0 failed" in line:
                        set_pi_light(f"{name}_cam0", "red")
                    if "cam1 failed" in line:
                        set_pi_light(f"{name}_cam1", "red")

        threading.Thread(target=reader_thread, args=(proc.stdout, "OUT"), daemon=True).start()
        threading.Thread(target=reader_thread, args=(proc.stderr, "ERR"), daemon=True).start()

        return proc
    
    except Exception as e:
        print_to_box(f"[{name}] SSH failed: {e}")
        set_pi_light(f"{name}_cam0", "red")
        set_pi_light(f"{name}_cam1", "red")
        return None

def run_remote_program():
    global procs, running
    procs = []
    try:
        with open("IP.txt") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 2:
                    name, ip = parts
                    proc = launch_for_host(name, ip)
                    if proc:
                        procs.append(proc)
    except FileNotFoundError:
        messagebox.showerror("Error", "IP.txt not found")
        return

    running = True
    while running:
        try:
            name, msg = q.get(timeout=0.5)
            print_to_box(f"[{name}] {msg}")
        except queue.Empty:
            continue

def start_program():
    global session_folder, consolidated_json_folder
    session_folder = datetime.now().strftime("%Y-%m-%d_%H-%M_test")
    consolidated_json_folder = None  # Reset consolidated JSON folder on new session
    threading.Thread(target=run_remote_program, daemon=True).start()

def collect_timestamp_files(local_folders, base_path):
    """
    Collect all timestamp.json files from downloaded folders into a single directory.
    Return the path to the consolidated folder.
    """
    #Create a consolidated folder for JSON files
    consolidated_folder = os.path.join(base_path, f"consolidated_{session_folder}")
    os.makedirs(consolidated_folder, exist_ok=True)

    print_to_box(f"\nCollecting timestamp.json files to: {consolidated_folder}")

    json_count = 0
    found_files = set() #Track found files to avoid duplicates

    for folder in local_folders:
        #Search for timestamp.json files in the downloaded folder
        #The SCP command copies the remote session folder into the local folder
        #So the structure is: local_folder/session_folder/timestamp.json
        json_patterns = [
            os.path.join(folder, session_folder, "*_timestamp*.json"),  #Most likely location
            os.path.join(folder, "*", "*_timestamp*.json"),
            os.path.join(folder, "**", "*_timestamp*.json"),
        ]

        for pattern in json_patterns:
            json_files = glob.glob(pattern, recursive=True)
            for json_file in json_files:
                if os.path.isfile(json_file) and json_file not in found_files:
                    found_files.add(json_file)

                    #Extract camera name from folder name
                    folder_name = os.path.basename(folder)
                    cam_name = folder_name.replace(f"_{session_folder}", "")    #Remove session suffix to get camera name

                    #Create unique filename to avoid overwriting
                    base_name = os.path.basename(json_file)
                    if not base_name.startswith(cam_name):
                        new_name = f"{cam_name}_{base_name}"
                    else:
                        new_name = base_name

                    dest_file = os.path.join(consolidated_folder, new_name)

                    try:
                        shutil.copy2(json_file, dest_file)
                        print_to_box(f" Copied: {new_name}")
                        json_count += 1
                        break  #Stop after first found file in this folder
                    except Exception as e:
                        print_to_box(f" Failed to copy {json_file}: {e}")

    print_to_box(f"Collected {json_count} timestamp.json files")
    return consolidated_folder if json_count > 0 else None

def stop_program():
    global running, consolidated_json_folder
    running = False
    print_to_box("Program terminating...")

    # first define hosts before using it
    try:
        with open("IP.txt") as f:
            hosts = [line.strip().split() for line in f if len(line.strip().split()) == 2]
    except FileNotFoundError:
        messagebox.showerror("Error", "IP.txt not found")
        return
    
    # Terminate processes
    for p in procs:
        try:
            p.terminate()
        except Exception:
            pass

    # Update lights for all cameras
    for cam_name, _ in hosts:
        set_pi_light(f"{cam_name}_cam0", "red")
        set_pi_light(f"{cam_name}_cam1", "red")

    print_to_box("All processes terminated.")

    #Pull data from Pis
    remote_base = "/home/terradynamics/Desktop/motion"
    default_path = os.path.expanduser("~/Desktop/Terradynamics")
    local_base = local_base_dir if local_base_dir else default_path

    #Track downloaded folders
    downloaded_folders = []

    for cam_name, ip in hosts:
        remote_folder = f"{remote_base}/{session_folder}"
        local_folder = os.path.join(local_base, f"{cam_name}_{session_folder}")
        os.makedirs(local_folder, exist_ok=True)

        print_to_box(f"[{cam_name}] pulling folder: {remote_folder}")
        scp_cmd = [
            "scp", "-r",
            f"terradynamics@{ip}:{remote_folder}",
            local_folder
        ]
        result = subprocess.call(scp_cmd)
        if result == 0:
            print_to_box(f"[{cam_name}] copied successfully to {local_folder}")
            downloaded_folders.append(local_folder)
        else:
            print_to_box(f"[{cam_name}] copy failed")

    #Automatically collect all timestamp.json files
    if downloaded_folders:
        consolidated_json_folder = collect_timestamp_files(downloaded_folders, local_base)
        if consolidated_json_folder:
            print_to_box(f"\n[OK] Timestamp files ready for pairing in: {consolidated_json_folder}")
            print_to_box("You can now click the PAIR button to align frames (or select a different folder)")

            #Update the pair button to show it's ready
            pair_btn.config(bg="green")
        else:
            print_to_box("\n[WARNING] No timestamp.json files found in downloaded data")
            consolidated_json_folder = None
    else:
        print_to_box("\n[WARNING] No data was successfully downloaded")
        consolidated_json_folder = None

def sort_idx(timestamp, ref, left=0, right=None):
    if right is None: right = len(ref)
    if left >= right: return left
    mid = (left + right) // 2
    return sort_idx(timestamp, ref, left, mid) if timestamp < ref[mid] else \
           sort_idx(timestamp, ref, mid + 1, right)

def frame_pairing(folder_path, threshold, interval):
    """ 
    Frame pairing function without capture functionality -
    capture is handled separately by the CAPTURE button
    """
    json_files = [f for f in os.listdir(folder_path) if f.endswith(".json")]

    if not json_files:
        messagebox.showwarning("Warning", "No JSON files found in the selected folder")
        return None
    
    data, timestamps = [], []
    for jf in json_files:
        with open(os.path.join(folder_path, jf), 'r', encoding='utf-8') as fp:
            data.append(json.load(fp))

    for d in data:
        timestamps.append(d.get("frames", []))

    max_ts, min_ts = datetime.min, datetime.max
    for ts_list in timestamps:
        for fr in ts_list:
            dt = datetime.strptime(fr["timestamp"], "%Y-%m-%d %H:%M:%S.%f")
            max_ts, min_ts = max(max_ts, dt), min(min_ts, dt)

    total = int((max_ts - min_ts).total_seconds() // interval) + 1
    reference = [min_ts + timedelta(seconds=i*interval) for i in range(total)]
    matrix = [['NA']*len(json_files) for _ in range(total)]

    for cam_idx, ts_list in enumerate(timestamps):
        for fr in ts_list:
            ts = datetime.strptime(fr["timestamp"], "%Y-%m-%d %H:%M:%S.%f")
            idx = sort_idx(ts, reference)
            nearest, best = None, float('inf')
            for k in (idx-1, idx, idx+1):
                if 0 <= k < len(reference):
                    diff = abs((ts - reference[k]).total_seconds())
                    if diff < best and diff <= threshold:
                        best, nearest = diff, k
            if nearest is not None and matrix[nearest][cam_idx] == 'NA':
                matrix[nearest][cam_idx] = fr["frame_index"]

    df = pd.DataFrame(matrix, columns=json_files)
    df.index.name = "Reference_Frame"
    output_file = os.path.join(folder_path, "aligned_frames.csv")
    df.to_csv(output_file, index=True)
    print_to_box(f"[OK] Frame alignment saved to: {output_file}")
    return df

def choose_folder():
    global consolidated_json_folder

    #Check if we have a consolidated folder from the last stop operation
    if consolidated_json_folder and os.path.exists(consolidated_json_folder):
        #Count JSON files in the consolidated folder
        json_count = len([f for f in os.listdir(consolidated_json_folder) if f.endswith(".json")])
        response = messagebox.askyesno(
            "Use Consolidated Folder?",
            f"Found {json_count} timestamp files from the last session.\n\nFolder: {consolidated_json_folder}\n\nClick 'Yes' to use these files or 'No' to select a different folder."
        )
        if response:
            folder = consolidated_json_folder
        else:
            folder = filedialog.askdirectory()
            if not folder: return
    else:
        folder = filedialog.askdirectory()
        if not folder: return

    try:
        th = float(threshold_entry.get())
        iv = float(interval_entry.get())
    except ValueError:
        messagebox.showerror("Error", "Threshold / Interval must be numbers")
        return
    
    print_to_box(f"\nProcessing folder: {folder}")
    result = frame_pairing(folder, th, iv)

    if result is not None:
        print_to_box("Frame pairing finished! Output saved to aligned_frames.csv")
        #Reset button color after succesful pairing
        pair_btn.config(bg="blue")
    else:
        print_to_box("Frame pairing failed - check if JSON files are present")

def choose_local_directory():
    global local_base_dir
    selected = filedialog.askdirectory()
    if selected:
        local_base_dir = selected
        local_dir_label.config(text=f"Selected: {selected}")

# GUI Setup
root = tk.Tk()
root.title("Run all Raspberry Pi")
root.geometry("1020x730")

# Copyright labels
tk.Label(root, text="Use for Terradynamics Lab only").pack()
tk.Label(root, text="Copyright: Pucheng Shao").pack()

setting = tk.LabelFrame(root, text="Settings", padx=10, pady=6)
setting.place(x=40, y=50, width=940, height=175) #increased height to fit focus setting

tk.Label(setting, text="Resolution:").grid(row=0, column=0, sticky="e")
res_var = tk.StringVar(value=DEFAULTS["resolution"])
tk.OptionMenu(setting, res_var, *RES_CHOICES).grid(row=0, column=1, sticky="w")

def add_entry(label, row, default_key):
    tk.Label(setting, text=label).grid(row=row, column=0, sticky="e")
    e = tk.Entry(setting, width=10)
    e.insert(0, DEFAULTS[default_key])
    e.grid(row=row, column=1, sticky="w")
    return e

fi_entry    = add_entry("Num of stored frames:",    1, "frame_interval")
fps_entry   = add_entry("Sec per frame:",     2, "timeFPS")
delay_entry = add_entry("Stop delay (s):",    3, "delay")
thres_entry = add_entry("Motion threshold:",  4, "motion_threshold")

#Add Focus Distance entry (row 5)
tk.Label(setting, text="Focus distance (m):").grid(row=5, column=0, sticky="e")
focus_entry = tk.Entry(setting, width=10)
focus_entry.insert(0, DEFAULTS["focus_distance"])
focus_entry.grid(row=5, column=1, sticky="w")

tk.Label(root, text="Image size").place(x=300, y=80)
tk.Label(root, text="The number of frames stored in the queue(used for writing frames into vedio before the motion detected) (5-10)").place(x=280, y=105)
tk.Label(root, text="Time interval between 2 adjacent frames (as small as possible(consider the performance of Pi))").place(x=280, y=130)
tk.Label(root, text="Time delay after no motion detected (1-3)").place(x=280, y=155)
tk.Label(root, text="Threshold for motion detection (0.005-0.02)").place(x=280, y=175)
tk.Label(root, text="The focus distance(in meters): 0.1=macro, 0.5=default, 2.0=medium, 10=far (higher value = farther focus)").place(x=280, y=198)

#Buttons
start_btn = tk.Button(root, text="START", width=20, height=2,
                      bg="green", fg="white", command=start_program)
start_btn.place(x=120, y=245)   #adjusted y position to fit new entry

select_dir_btn = tk.Button(root, text="Select Save Location", command=choose_local_directory)
select_dir_btn.place(x=120, y=295)  #adjusted y position to fit new entry

local_dir_label = tk.Label(root, text="Selected: (Default = ~/Desktop/Terradynamics)", anchor="w")
local_dir_label.place(x=20, y=325)  #adjusted y position to fit new entry

stop_btn = tk.Button(root, text="STOP", width=20, height=2,
                     bg="red", fg="white", command=stop_program)
stop_btn.place(x=360, y=245)    #adjusted y position to fit new entry

pair_btn = tk.Button(root, text="PAIR", width=20, height=2,
                     bg="blue", fg="white", command=choose_folder)
pair_btn.place(x=600, y=245)    #adjusted y position to fit new entry

# Add CAPTURE button
capture_btn = tk.Button(root, text="CAPTURE", width=20, height=2,
                        bg="orange", fg="black", command=run_capture_thread)
capture_btn.place(x=840, y=245)  

#Threshold and Interval settings
tk.Label(root, text="Threshold (s):").place(x=300, y=305)   #adjusted y position
threshold_entry = tk.Entry(root, width=8)
threshold_entry.insert(0, "0.2")
threshold_entry.place(x=400, y=305) #adjusted y position
tk.Label(root, text="The threshold used to determine whether two frames can be paired").place(x=470, y=305) #adjusted y position

tk.Label(root, text="Interval (s):").place(x=315, y=335)    #adjusted y position
interval_entry = tk.Entry(root, width=8)
interval_entry.insert(0, "1.2")
interval_entry.place(x=400, y=335)  #adjusted y position
tk.Label(root, text="Normally, this interval should be as same as the 'Sec per frame' in settings").place(x=470, y=335) #adjusted y position

output_box = tk.Text(root, width=90, height=21)
output_box.place(x=40, y=375)   #adjusted y position

"""light"""
pi_status = {}  # {pi_name: {"canvas": ..., "circle": ...}}

status_frame = tk.LabelFrame(root, text="Pi state", padx=10, pady=10)
status_frame.place(x=720, y=375, width=260, height=300) #adjusted y position

def create_cube_lights(pi_names):
    canvas_size = 240
    canvas = tk.Canvas(status_frame, width=canvas_size, height=canvas_size, bg="white")
    canvas.pack()

    cube_points = {
        0: (60, 60),   # top-front-left
        1: (140, 60),  # top-front-right
        2: (60, 140),  # top-back-left
        3: (140, 140), # top-back-right
        4: (90, 90),   # bottom-front-left
        5: (170, 90),  # bottom-front-right
        6: (90, 170),  # bottom-back-left
        7: (170, 170), # bottom-back-right
    }

    edges = [
        (0, 1), (1, 3), (3, 2), (2, 0), # top face
        (4, 5), (5, 7), (7, 6), (6, 4), # bottom face
        (0, 4), (1, 5), (2, 6), (3, 7)  # vertical edges
    ]
    for a, b in edges:
        x1, y1 = cube_points[a]
        x2, y2 = cube_points[b]
        canvas.create_line(x1, y1, x2, y2, fill="black")

    for i, pi in enumerate(pi_names[:8]):
        x, y = cube_points[i]

        dx0, dy0 = 0, 0
        cam0_id = f"{pi}_cam0"
        circle0 = canvas.create_oval(x+dx0-5, y+dy0-5, x+dx0+5, y+dy0+5, fill="gray")
        canvas.create_text(x+dx0, y+dy0-10, text=cam0_id, font=("Arial", 6))
        pi_status[cam0_id] = {"canvas": canvas, "circle": circle0}

        dx1, dy1 = 10, 10
        cam1_id = f"{pi}_cam1"
        circle1 = canvas.create_oval(x+dx1-5, y+dy1-5, x+dx1+5, y+dy1+5, fill="gray")
        canvas.create_text(x+dx1, y+dy1+10, text=cam1_id, font=("Arial", 6))
        pi_status[cam1_id] = {"canvas": canvas, "circle": circle1}

def set_pi_light(pi, color):
    if pi in pi_status:
        canvas = pi_status[pi]["canvas"]
        circle = pi_status[pi]["circle"]
        # Use root.after to ensure thread-safe GUI updates
        root.after(0, lambda: canvas.itemconfig(circle, fill=color))

def init_pi_lights():
    try:
        with open("IP.txt") as f:
            pi_names = [line.strip().split()[0] for line in f if len(line.strip().split()) == 2]
            if pi_names:
                create_cube_lights(pi_names)
                print(f"Initialized cube visualization for {len(pi_names)} Pis: {pi_names}")
            else:
                print("No valid Pi entries found in IP.txt")
                # Create a demo cube if no IPs found
                create_cube_lights(["pi1", "pi2", "pi3", "pi4"])
    except FileNotFoundError:
        print("IP.txt not found - creating demo cube visualization")
        # Create a demo cube with sample Pi names
        create_cube_lights(["pi1", "pi2", "pi3", "pi4"])
    except Exception as e:
        print(f"Error initializing Pi lights: {e}")
        # Create a demo cube as fallback
        create_cube_lights(["pi1", "pi2", "pi3", "pi4"])

init_pi_lights()

root.mainloop()
