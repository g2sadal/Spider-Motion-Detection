#!/usr/bin/env python3
import tkinter as tk
import subprocess, threading, queue, os, json
from datetime import datetime, timedelta
import pandas as pd
from tkinter import filedialog, messagebox
import time

local_base_dir = None  
session_folder = ""



DEFAULTS = {
    "resolution":        "4608x2592",
    "frame_interval":    "10",
    "timeFPS":           "1.2",
    "delay":             "2",
    "motion_threshold":  "0.005"
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

def build_remote_cmd():

    res   = res_var.get()
    fi    = fi_entry.get().strip()
    tpf   = fps_entry.get().strip()
    dly   = delay_entry.get().strip()
    thr   = thres_entry.get().strip()

    cmd = ["python3", "-u", "/home/terradynamics/Desktop/motion/dual_cam_v11.py",
           "--resolution", res]

    if fi:  cmd += ["--frame_interval", fi]
    if tpf: cmd += ["--timeFPS", tpf]
    if dly: cmd += ["--delay", dly]
    if thr: cmd += ["--motion_threshold", thr]

    return " ".join(cmd)

def launch_for_host(name, ip):
    remote_cmd = build_remote_cmd()
    try:
        proc = subprocess.Popen(
            ["ssh", f"terradynamics@{ip}", remote_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        threading.Thread(target=reader, args=(name, proc.stdout), daemon=True).start()
        set_pi_light(name, "green")
        return proc
    except Exception:
        set_pi_light(name, "red")
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
                    procs.append(launch_for_host(name, ip))
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
    global session_folder
    session_folder = datetime.now().strftime("%Y-%m-%d_%H-%M_test")
    threading.Thread(target=run_remote_program, daemon=True).start()

def stop_program():
    global running
    running = False
    print_to_box("Program terminating...")
    for p in procs:
        try:
            p.terminate()
            for cam_name, _ in hosts:
                set_pi_light(cam_name, "red")

        except Exception:
            pass
    print_to_box("All processes terminated.")

    try:
        with open("IP.txt") as f:
            hosts = [line.strip().split() for line in f if len(line.strip().split()) == 2]
    except FileNotFoundError:
        messagebox.showerror("Error", "IP.txt not found")
        return

    remote_base = "/home/terradynamics/Desktop/motion"
    default_path = os.path.expanduser("~/Desktop/Terradynamics")
    local_base = local_base_dir if local_base_dir else default_path


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
        else:
            print_to_box(f"[{cam_name}] copy failed")

def sort_idx(timestamp, ref, left=0, right=None):
    if right is None: right = len(ref)
    if left >= right: return left
    mid = (left + right) // 2
    return sort_idx(timestamp, ref, left, mid) if timestamp < ref[mid] else \
           sort_idx(timestamp, ref, mid + 1, right)

def frame_pairing(folder_path, threshold, interval, remote_capture_dir="/home/terradynamics/captures",
                  capture_script="/home/terradynamics/Desktop/motion/img_capture.py"):
    try:
        with open("IP.txt") as f:
            hosts = [line.strip().split() for line in f if len(line.strip().split()) == 2]
    except FileNotFoundError:
        messagebox.showerror("Error", "IP.txt not found")
        return

    ts_prefix = datetime.now().strftime("%Y%m%d_%H%M%S")   

    for cam_name, ip in hosts:
        ssh_cmd = [
            "ssh", f"terradynamics@{ip}",
            f"python3 {capture_script} --out_dir {remote_capture_dir}"
        ]
        if subprocess.call(ssh_cmd) == 0:
            print_to_box(f"[{cam_name}] capture done (saved on Pi)")
        else:
            print_to_box(f"[{cam_name}] capture failed")

    
    json_files = [f for f in os.listdir(folder_path) if f.endswith(".json")]
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
            for k in (idx-1, idx):
                if 0 <= k < len(reference):
                    diff = abs((ts - reference[k]).total_seconds())
                    if diff < best and diff <= threshold:
                        best, nearest = diff, k
            if nearest is not None and matrix[nearest][cam_idx] == 'NA':
                matrix[nearest][cam_idx] = fr["frame_index"]

    df = pd.DataFrame(matrix, columns=json_files)
    df.index.name = "Reference_Frame"
    df.to_csv(os.path.join(folder_path, "aligned_frames.csv"), index=True)
    return df

def choose_folder():
    folder = filedialog.askdirectory()
    if not folder: return
    try:
        th = float(threshold_entry.get())
        iv = float(interval_entry.get())
    except ValueError:
        messagebox.showerror("Error", "Threshold / Interval must be numbers")
        return
    frame_pairing(folder, th, iv)
    print_to_box("Frame pairing finished! Output saved to aligned_frames.csv")

def choose_local_directory():
    global local_base_dir
    selected = filedialog.askdirectory()
    if selected:
        local_base_dir = selected
        local_dir_label.config(text=f"Selected: {selected}")



root = tk.Tk()
root.title("Run all Raspberry Pi")
root.geometry("1020x730")

tk.Label(root, text="Use for Terradynamics Lab only").pack()
tk.Label(root, text="Copyright: Pucheng Shao").pack()

setting = tk.LabelFrame(root, text="Settings", padx=10, pady=6)
setting.place(x=40, y=50, width=940, height=150)

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
tk.Label(root, text="Image size").place(x=300, y=80)
tk.Label(root, text="The number of frames stored in the queue(used for writing frames into vedio before the motion detected)").place(x=280, y=105)
tk.Label(root, text="Time interval between 2 adjacent frames").place(x=280, y=130)
tk.Label(root, text="Time delay after no motion detected").place(x=280, y=155)
tk.Label(root, text="Threshold for motion detection").place(x=280, y=175)
start_btn = tk.Button(root, text="START", width=20, height=2,
                      bg="green", fg="white", command=start_program)
start_btn.place(x=120, y=220)

select_dir_btn = tk.Button(root, text="Select Save Location", command=choose_local_directory)
select_dir_btn.place(x=120, y=270)

local_dir_label = tk.Label(root, text="Selected: (Default = ~/Desktop/Terradynamics)", anchor="w")
local_dir_label.place(x=20, y=300)

stop_btn = tk.Button(root, text="STOP", width=20, height=2,
                     bg="red", fg="white", command=stop_program)
stop_btn.place(x=360, y=220)

pair_btn = tk.Button(root, text="PAIR", width=20, height=2,
                     bg="blue", fg="white", command=choose_folder)
pair_btn.place(x=600, y=220)

tk.Label(root, text="Threshold (s):").place(x=300, y=280)
threshold_entry = tk.Entry(root, width=8)
threshold_entry.insert(0, "0.2")
threshold_entry.place(x=400, y=280)
tk.Label(root, text="The threshold used to determine whether two frames can be paired").place(x=470, y=280)

tk.Label(root, text="Interval (s):").place(x=315, y=310)
interval_entry = tk.Entry(root, width=8)
interval_entry.insert(0, "1.2")
interval_entry.place(x=400, y=310)
tk.Label(root, text="Normally, this interval should be as same as the 'Sec per frame' in settings").place(x=470, y=310)

output_box = tk.Text(root, width=90, height=22)
output_box.place(x=40, y=350)


"""light"""

pi_status = {}  # {pi_name: {"canvas": ..., "circle": ...}}

status_frame = tk.LabelFrame(root, text="Pi state", padx=10, pady=10)
status_frame.place(x=720, y=350, width=260, height=200)

def create_status_lights(pi_names):
    for i, pi in enumerate(pi_names):
        tk.Label(status_frame, text=pi).grid(row=i, column=0, sticky="w")
        canvas = tk.Canvas(status_frame, width=20, height=20)
        canvas.grid(row=i, column=1)
        circle = canvas.create_oval(2, 2, 18, 18, fill="gray")
        pi_status[pi] = {"canvas": canvas, "circle": circle}

def set_pi_light(pi, color):
    if pi in pi_status:
        canvas = pi_status[pi]["canvas"]
        circle = pi_status[pi]["circle"]
        canvas.itemconfig(circle, fill=color)

def init_pi_lights():
    try:
        with open("IP.txt") as f:
            pi_names = [line.strip().split()[0] for line in f if len(line.strip().split()) == 2]
            create_status_lights(pi_names)
    except:
        pass

init_pi_lights()


root.mainloop()
