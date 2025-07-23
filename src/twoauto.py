import numpy as np
import matplotlib as mpl, matplotlib.pyplot as plt
import time, os, threading
# import modules from other python files
from data_process import data, live_data
from arduino_manager import arduino
from moment_data_process import data_frame

plt.rcParams['axes.grid'] = True
plt.rcParams["figure.autolayout"] = True
prop_cycle = plt.rcParams['axes.prop_cycle']
colors = prop_cycle.by_key()['color']
mpl.use('TkAgg')

# Initialisation of some constants and variables
port = 'COM4'
baudrate = 230400
TRIAL_DURATION_SECONDS = 45 #  maximum duration for a single PID trial

# --- PID Parameter Sets for Automated Testing ---
PID_PARAM_SETS = [
    "700.0,490.0,2.75,-0.04,0.0,-0.008",
    "450.0,315.0,4.00,-0.030,0,-0.0117",
    "450.0,315.0,4.00,-0.030,0,-0.0150",
    "450.0,315.0,4.00,-0.043,0,-0.0050",
    "450.0,315.0,4.00,-0.043,0,-0.0083",
    "450.0,315.0,4.00,-0.043,0,-0.0117",
    "450.0,315.0,4.00,-0.043,0,-0.0150",
    "450.0,315.0,4.00,-0.057,0,-0.0050",
    "450.0,315.0,4.00,-0.057,0,-0.0083",
    "450.0,315.0,4.00,-0.057,0,-0.0117",
    "450.0,315.0,4.00,-0.057,0,-0.0150",
    "450.0,315.0,4.00,-0.070,0,-0.0050",
    "450.0,315.0,4.00,-0.070,0,-0.0083",
    "450.0,315.0,4.00,-0.070,0,-0.0117",
    "450.0,315.0,4.00,-0.070,0,-0.0150",
    "450.0,315.0,4.50,-0.030,0,-0.0050",
    "450.0,315.0,4.50,-0.030,0,-0.0083",
    "450.0,315.0,4.50,-0.030,0,-0.0117",
    "450.0,315.0,4.50,-0.030,0,-0.0150",
    "450.0,315.0,4.50,-0.043,0,-0.0050",
    "450.0,315.0,4.50,-0.043,0,-0.0083",
    "450.0,315.0,4.50,-0.043,0,-0.0117",
    "450.0,315.0,4.50,-0.043,0,-0.0150",
    "450.0,315.0,4.50,-0.057,0,-0.0050",
    "450.0,315.0,4.50,-0.057,0,-0.0083",
    "450.0,315.0,4.50,-0.057,0,-0.0117",
    "450.0,315.0,4.50,-0.057,0,-0.0150",
    "450.0,315.0,4.50,-0.070,0,-0.0050",
    "450.0,315.0,4.50,-0.070,0,-0.0083",
    "450.0,315.0,4.50,-0.070,0,-0.0117",
    "450.0,315.0,4.50,-0.070,0,-0.0150",
    "525.0,367.5,3.00,-0.030,0,-0.0050",
    "525.0,367.5,3.00,-0.030,0,-0.0083",
    "525.0,367.5,3.00,-0.030,0,-0.0117",
    "525.0,367.5,3.00,-0.030,0,-0.0150",
    "525.0,367.5,3.00,-0.043,0,-0.0050",
    "525.0,367.5,3.00,-0.043,0,-0.0083",
    "525.0,367.5,3.00,-0.043,0,-0.0117",
    "525.0,367.5,3.00,-0.043,0,-0.0150",
    "525.0,367.5,3.00,-0.057,0,-0.0050",
    "525.0,367.5,3.00,-0.057,0,-0.0083",
    "525.0,367.5,3.00,-0.057,0,-0.0117",
    "525.0,367.5,3.00,-0.057,0,-0.0150",
    "525.0,367.5,3.00,-0.070,0,-0.0050",
    "525.0,367.5,3.00,-0.070,0,-0.0083",
    "525.0,367.5,3.00,-0.070,0,-0.0117",
    "525.0,367.5,3.00,-0.070,0,-0.0150",
    "525.0,367.5,3.50,-0.030,0,-0.0050",
    "525.0,367.5,3.50,-0.030,0,-0.0083",
    "525.0,367.5,3.50,-0.030,0,-0.0117",
    "525.0,367.5,3.50,-0.030,0,-0.0150",
    "525.0,367.5,3.50,-0.043,0,-0.0050",
    "525.0,367.5,3.50,-0.043,0,-0.0083",
    "525.0,367.5,3.50,-0.043,0,-0.0117",
    "525.0,367.5,3.50,-0.043,0,-0.0150",
    "525.0,367.5,3.50,-0.057,0,-0.0050",
    "525.0,367.5,3.50,-0.057,0,-0.0083",
    "525.0,367.5,3.50,-0.057,0,-0.0117",
    "525.0,367.5,3.50,-0.057,0,-0.0150",
    "525.0,367.5,3.50,-0.070,0,-0.0050",
    "525.0,367.5,3.50,-0.070,0,-0.0083",
    "525.0,367.5,3.50,-0.070,0,-0.0117",
    "525.0,367.5,3.50,-0.070,0,-0.0150",
    "525.0,367.5,4.00,-0.030,0,-0.0050",
    "525.0,367.5,4.00,-0.030,0,-0.0083",
    "525.0,367.5,4.00,-0.030,0,-0.0117",
    "525.0,367.5,4.00,-0.030,0,-0.0150",
    "525.0,367.5,4.00,-0.043,0,-0.0050",
    "525.0,367.5,4.00,-0.043,0,-0.0083",
    "525.0,367.5,4.00,-0.043,0,-0.0117",
    "525.0,367.5,4.00,-0.043,0,-0.0150",
    "525.0,367.5,4.00,-0.057,0,-0.0050",
    "525.0,367.5,4.00,-0.057,0,-0.0083",
    "525.0,367.5,4.00,-0.057,0,-0.0117",
    "525.0,367.5,4.00,-0.057,0,-0.0150",
    "525.0,367.5,4.00,-0.070,0,-0.0050",
    "525.0,367.5,4.00,-0.070,0,-0.0083",
    "525.0,367.5,4.00,-0.070,0,-0.0117",
    "525.0,367.5,4.00,-0.070,0,-0.0150",
    "525.0,367.5,4.50,-0.030,0,-0.0050",
    "525.0,367.5,4.50,-0.030,0,-0.0083",
    "525.0,367.5,4.50,-0.030,0,-0.0117",
    "525.0,367.5,4.50,-0.030,0,-0.0150",
    "525.0,367.5,4.50,-0.043,0,-0.0050",
    "525.0,367.5,4.50,-0.043,0,-0.0083",
    "525.0,367.5,4.50,-0.043,0,-0.0117",
    "525.0,367.5,4.50,-0.043,0,-0.0150",
    "525.0,367.5,4.50,-0.057,0,-0.0050",
    "525.0,367.5,4.50,-0.057,0,-0.0083",
    "525.0,367.5,4.50,-0.057,0,-0.0117",
    "525.0,367.5,4.50,-0.057,0,-0.0150",
    "525.0,367.5,4.50,-0.070,0,-0.0050",
    "525.0,367.5,4.50,-0.070,0,-0.0083",
    "525.0,367.5,4.50,-0.070,0,-0.0117",
    "525.0,367.5,4.50,-0.070,0,-0.0150",
    "600.0,420.0,3.00,-0.030,0,-0.0050",
    "600.0,420.0,3.00,-0.030,0,-0.0083",
    "600.0,420.0,3.00,-0.030,0,-0.0117",
    "600.0,420.0,3.00,-0.030,0,-0.0150",
    "600.0,420.0,3.00,-0.043,0,-0.0050",
    "600.0,420.0,3.00,-0.043,0,-0.0083",
    "600.0,420.0,3.00,-0.043,0,-0.0117",
    "600.0,420.0,3.00,-0.043,0,-0.0150",
    "600.0,420.0,3.00,-0.057,0,-0.0050",
    "600.0,420.0,3.00,-0.057,0,-0.0083",
    "600.0,420.0,3.00,-0.057,0,-0.0117",
    "600.0,420.0,3.00,-0.057,0,-0.0150",
    "600.0,420.0,3.00,-0.070,0,-0.0050",
    "600.0,420.0,3.00,-0.070,0,-0.0083",
    "600.0,420.0,3.00,-0.070,0,-0.0117",
    "600.0,420.0,3.00,-0.070,0,-0.0150",
    "600.0,420.0,3.50,-0.030,0,-0.0050",
    "600.0,420.0,3.50,-0.030,0,-0.0083",
    "600.0,420.0,3.50,-0.030,0,-0.0117",
    "600.0,420.0,3.50,-0.030,0,-0.0150",
    "600.0,420.0,3.50,-0.043,0,-0.0050",
    "600.0,420.0,3.50,-0.043,0,-0.0083",
    "600.0,420.0,3.50,-0.043,0,-0.0117",
    "600.0,420.0,3.50,-0.043,0,-0.0150",
    "600.0,420.0,3.50,-0.057,0,-0.0050",
    "600.0,420.0,3.50,-0.057,0,-0.0083",
    "600.0,420.0,3.50,-0.057,0,-0.0117",
    "600.0,420.0,3.50,-0.057,0,-0.0150",
    "600.0,420.0,3.50,-0.070,0,-0.0050",
    "600.0,420.0,3.50,-0.070,0,-0.0083",
    "600.0,420.0,3.50,-0.070,0,-0.0117",
    "600.0,420.0,3.50,-0.070,0,-0.0150",
    "600.0,420.0,4.00,-0.030,0,-0.0050",
    "600.0,420.0,4.00,-0.030,0,-0.0083",
    "600.0,420.0,4.00,-0.030,0,-0.0117",
    "600.0,420.0,4.00,-0.030,0,-0.0150",
    "600.0,420.0,4.00,-0.043,0,-0.0050",
    "600.0,420.0,4.00,-0.043,0,-0.0083",
    "600.0,420.0,4.00,-0.043,0,-0.0117",
    "600.0,420.0,4.00,-0.043,0,-0.0150",
    "600.0,420.0,4.00,-0.057,0,-0.0050",
    "600.0,420.0,4.00,-0.057,0,-0.0083",
    "600.0,420.0,4.00,-0.057,0,-0.0117",
    "600.0,420.0,4.00,-0.057,0,-0.0150",
    "600.0,420.0,4.00,-0.070,0,-0.0050",
    "600.0,420.0,4.00,-0.070,0,-0.0083",
    "600.0,420.0,4.00,-0.070,0,-0.0117",
    "600.0,420.0,4.00,-0.070,0,-0.0150",
    "600.0,420.0,4.50,-0.030,0,-0.0050",
    "600.0,420.0,4.50,-0.030,0,-0.0083",
    "600.0,420.0,4.50,-0.030,0,-0.0117",
    "600.0,420.0,4.50,-0.030,0,-0.0150",
    "600.0,420.0,4.50,-0.043,0,-0.0050",
    "600.0,420.0,4.50,-0.043,0,-0.0083",
    "600.0,420.0,4.50,-0.043,0,-0.0117",
    "600.0,420.0,4.50,-0.043,0,-0.0150",
    "600.0,420.0,4.50,-0.057,0,-0.0050",
    "600.0,420.0,4.50,-0.057,0,-0.0083",
    "600.0,420.0,4.50,-0.057,0,-0.0117",
    "600.0,420.0,4.50,-0.057,0,-0.0150",
    "600.0,420.0,4.50,-0.070,0,-0.0050",
    "600.0,420.0,4.50,-0.070,0,-0.0083",
    "600.0,420.0,4.50,-0.070,0,-0.0117",
    "600.0,420.0,4.50,-0.070,0,-0.0150",
    "675.0,472.5,3.00,-0.030,0,-0.0050",
    "675.0,472.5,3.00,-0.030,0,-0.0083",
    "675.0,472.5,3.00,-0.030,0,-0.0117",
    "675.0,472.5,3.00,-0.030,0,-0.0150",
    "675.0,472.5,3.00,-0.043,0,-0.0050",
    "675.0,472.5,3.00,-0.043,0,-0.0083",
    "675.0,472.5,3.00,-0.043,0,-0.0117",
    "675.0,472.5,3.00,-0.043,0,-0.0150",
    "675.0,472.5,3.00,-0.057,0,-0.0050",
    "675.0,472.5,3.00,-0.057,0,-0.0083",
    "675.0,472.5,3.00,-0.057,0,-0.0117",
    "675.0,472.5,3.00,-0.057,0,-0.0150",
    "675.0,472.5,3.00,-0.070,0,-0.0050",
    "675.0,472.5,3.00,-0.070,0,-0.0083",
    "675.0,472.5,3.00,-0.070,0,-0.0117",
    "675.0,472.5,3.00,-0.070,0,-0.0150",
    "675.0,472.5,3.50,-0.030,0,-0.0050",
    "675.0,472.5,3.50,-0.030,0,-0.0083",
    "675.0,472.5,3.50,-0.030,0,-0.0117",
    "675.0,472.5,3.50,-0.030,0,-0.0150",
    "675.0,472.5,3.50,-0.043,0,-0.0050",
    "675.0,472.5,3.50,-0.043,0,-0.0083",
    "675.0,472.5,3.50,-0.043,0,-0.0117",
    "675.0,472.5,3.50,-0.043,0,-0.0150",
    "675.0,472.5,3.50,-0.057,0,-0.0050",
    "675.0,472.5,3.50,-0.057,0,-0.0083",
    "675.0,472.5,3.50,-0.057,0,-0.0117",
    "675.0,472.5,3.50,-0.057,0,-0.0150",
    "675.0,472.5,3.50,-0.070,0,-0.0050",
    "675.0,472.5,3.50,-0.070,0,-0.0083",
    "675.0,472.5,3.50,-0.070,0,-0.0117",
    "675.0,472.5,3.50,-0.070,0,-0.0150",
    "675.0,472.5,4.00,-0.030,0,-0.0050",
    "675.0,472.5,4.00,-0.030,0,-0.0083",
    "675.0,472.5,4.00,-0.030,0,-0.0117",
    "675.0,472.5,4.00,-0.030,0,-0.0150",
    "675.0,472.5,4.00,-0.043,0,-0.0050",
    "675.0,472.5,4.00,-0.043,0,-0.0083",
    "675.0,472.5,4.00,-0.043,0,-0.0117",
    "675.0,472.5,4.00,-0.043,0,-0.0150",
    "675.0,472.5,4.00,-0.057,0,-0.0050",
    "675.0,472.5,4.00,-0.057,0,-0.0083",
    "675.0,472.5,4.00,-0.057,0,-0.0117",
    "675.0,472.5,4.00,-0.057,0,-0.0150",
    "675.0,472.5,4.00,-0.070,0,-0.0050",
    "675.0,472.5,4.00,-0.070,0,-0.0083",
    "675.0,472.5,4.00,-0.070,0,-0.0117",
    "675.0,472.5,4.00,-0.070,0,-0.0150",
    "675.0,472.5,4.50,-0.030,0,-0.0050",
    "675.0,472.5,4.50,-0.030,0,-0.0083",
    "675.0,472.5,4.50,-0.030,0,-0.0117",
    "675.0,472.5,4.50,-0.030,0,-0.0150",
    "675.0,472.5,4.50,-0.043,0,-0.0050",
    "675.0,472.5,4.50,-0.043,0,-0.0083",
    "675.0,472.5,4.50,-0.043,0,-0.0117",
    "675.0,472.5,4.50,-0.043,0,-0.0150",
    "675.0,472.5,4.50,-0.057,0,-0.0050",
    "675.0,472.5,4.50,-0.057,0,-0.0083",
    "675.0,472.5,4.50,-0.057,0,-0.0117",
    "675.0,472.5,4.50,-0.057,0,-0.0150",
    "675.0,472.5,4.50,-0.070,0,-0.0050",
    "675.0,472.5,4.50,-0.070,0,-0.0083",
    "675.0,472.5,4.50,-0.070,0,-0.0117",
    "675.0,472.5,4.50,-0.070,0,-0.0150",
    "750.0,525.0,3.00,-0.030,0,-0.0050",
    "750.0,525.0,3.00,-0.030,0,-0.0083",
    "750.0,525.0,3.00,-0.030,0,-0.0117",
    "750.0,525.0,3.00,-0.030,0,-0.0150",
    "750.0,525.0,3.00,-0.043,0,-0.0050",
    "750.0,525.0,3.00,-0.043,0,-0.0083",
    "750.0,525.0,3.00,-0.043,0,-0.0117",
    "750.0,525.0,3.00,-0.043,0,-0.0150",
    "750.0,525.0,3.00,-0.057,0,-0.0050",
    "750.0,525.0,3.00,-0.057,0,-0.0083",
    "750.0,525.0,3.00,-0.057,0,-0.0117",
    "750.0,525.0,3.00,-0.057,0,-0.0150",
    "750.0,525.0,3.00,-0.070,0,-0.0050",
    "750.0,525.0,3.00,-0.070,0,-0.0083",
    "750.0,525.0,3.00,-0.070,0,-0.0117",
    "750.0,525.0,3.00,-0.070,0,-0.0150",
    "750.0,525.0,3.50,-0.030,0,-0.0050",
    "750.0,525.0,3.50,-0.030,0,-0.0083",
    "750.0,525.0,3.50,-0.030,0,-0.0117",
    "750.0,525.0,3.50,-0.030,0,-0.0150",
    "750.0,525.0,3.50,-0.043,0,-0.0050",
    "750.0,525.0,3.50,-0.043,0,-0.0083",
    "750.0,525.0,3.50,-0.043,0,-0.0117",
    "750.0,525.0,3.50,-0.043,0,-0.0150",
    "750.0,525.0,3.50,-0.057,0,-0.0050",
    "750.0,525.0,3.50,-0.057,0,-0.0083",
    "750.0,525.0,3.50,-0.057,0,-0.0117",
    "750.0,525.0,3.50,-0.057,0,-0.0150",
    "750.0,525.0,3.50,-0.070,0,-0.0050",
    "750.0,525.0,3.50,-0.070,0,-0.0083",
    "750.0,525.0,3.50,-0.070,0,-0.0117",
    "750.0,525.0,3.50,-0.070,0,-0.0150",
    "750.0,525.0,4.00,-0.030,0,-0.0050",
    "750.0,525.0,4.00,-0.030,0,-0.0083",
    "750.0,525.0,4.00,-0.030,0,-0.0117",
    "750.0,525.0,4.00,-0.030,0,-0.0150",
    "750.0,525.0,4.00,-0.043,0,-0.0050",
    "750.0,525.0,4.00,-0.043,0,-0.0083",
    "750.0,525.0,4.00,-0.043,0,-0.0117",
    "750.0,525.0,4.00,-0.043,0,-0.0150",
    "750.0,525.0,4.00,-0.057,0,-0.0050",
    "750.0,525.0,4.00,-0.057,0,-0.0083",
    "750.0,525.0,4.00,-0.057,0,-0.0117",
    "750.0,525.0,4.00,-0.057,0,-0.0150",
    "750.0,525.0,4.00,-0.070,0,-0.0050",
    "750.0,525.0,4.00,-0.070,0,-0.0083",
    "750.0,525.0,4.00,-0.070,0,-0.0117",
    "750.0,525.0,4.00,-0.070,0,-0.0150",
    "750.0,525.0,4.50,-0.030,0,-0.0050",
    "750.0,525.0,4.50,-0.030,0,-0.0083",
    "750.0,525.0,4.50,-0.030,0,-0.0117",
    "750.0,525.0,4.50,-0.030,0,-0.0150",
    "750.0,525.0,4.50,-0.043,0,-0.0050",
    "750.0,525.0,4.50,-0.043,0,-0.0083",
    "750.0,525.0,4.50,-0.043,0,-0.0117",
    "750.0,525.0,4.50,-0.043,0,-0.0150",
    "750.0,525.0,4.50,-0.057,0,-0.0050",
    "750.0,525.0,4.50,-0.057,0,-0.0083",
    "750.0,525.0,4.50,-0.057,0,-0.0117",
    "750.0,525.0,4.50,-0.057,0,-0.0150",
    "750.0,525.0,4.50,-0.070,0,-0.0050",
    "750.0,525.0,4.50,-0.070,0,-0.0083",
    "750.0,525.0,4.50,-0.070,0,-0.0117",
    "750.0,525.0,4.50,-0.070,0,-0.0150",
]

# Safety check to ensure the list has been populated
if not PID_PARAM_SETS or "PASTE" in PID_PARAM_SETS[0]:
    print("❌ ERROR: The PID_PARAM_SETS list is empty or contains the placeholder text.")
    print("Please run the 'generate_pid_sets.py' script and paste the output into this list before running.")
    exit()

print(f"✅ Loaded {len(PID_PARAM_SETS)} PID parameter sets for testing.")

class cart_pendulum():
    '''Cart pendulum manager class'''
    def __init__(
        self,
        arduino,
        data,
        temp_data,
        data_frame,):
        
        self.arduino = arduino
        self.data = data
        self.temp_datum = temp_data
        self.df = data_frame
        self.module_name = r"\Defaut_Cart_Pendulum"
        self.center_count = 0
        self.distance = 0
        self.NR_counter = 0
        self.thread_counter = 0
        self._pid_trial_start_time = 0
        self.flag_list = {
            "command": True, "reset": False, "center": False, "pid": False, "measure": False,
            "NR": False, "setSpeed": False, "freq_scan": False, "multi_freq": False,
            "omega": True, "amp": True, "amp_0": True, "swing_request": True,
            "pid_input": True, "thread_init": True, "flag_scan": True, "setSpeed_request": True,
        }
        self.init_true_flag_list = ["command", "omega", "amp", "amp_0", "swing_request", "pid_input", "thread_init", "flag_scan", "setSpeed_request"]
        self.init_false_flag_list = ["reset", "center", "pid", "measure", "NR", "setSpeed", "freq_scan", "multi_freq"]
        self.reset_dict = { "Resetting...", "No command detected.", "Unidentified command. Please try again.", "More than one command detected. Resetting the values.", "Hasn't been centred. Please centre the cart first.", "Terminating since limit switch pressed..."}
        self.command_dict = {"Beginning centring.": "center", "Beginning PID control.": "pid", "Beginning measuring the natural frequency and quality factor.": "measure", "Beginning the normalised resonance.": "NR", "Beginning setting the speed and acceleration.": "setSpeed", "Beginning the frequency scan.": "freq_scan"}
        self._current_pid_param_index = 0

    def reset_flag_list(self, swing_request=False):
        for flag in self.init_true_flag_list: self.flag_list[flag] = True
        for flag in self.init_false_flag_list: self.flag_list[flag] = False
        if swing_request: self.flag_list["swing_request"] = False

    def reconnect(self, exp=False, send_terminate=False):
        '''This function now only handles closing the connection and saving data.'''
        if send_terminate:
            time.sleep(0.1)
            if hasattr(self.arduino, 'board') and self.arduino.board.is_open:
                self.arduino.send_message("Terminate\n")
        try:
            # Set the flag to prevent the event handler race condition
            self.temp_datum.programmatic_close = True
            plt.close("all")
            
            if hasattr(self.arduino, 'board') and self.arduino.board.is_open:
                self.arduino.read_single()
                self.arduino.clear()
                self.arduino.board.close()
                print("Arduino connection closed.")
            
            if exp:
                self.temp_datum.export_csv(self.module_name, input_spec_info=True)
        except KeyboardInterrupt:
            if hasattr(self.arduino, 'board') and self.arduino.board.is_open: self.arduino.board.close()
            print("KeyboardInterrupt during reconnect. Exiting.")
            exit()

    def reset(self, reset_data=True):
        self.arduino.clear()
        self.reset_flag_list()
        self.data.clear_data()
        self.data.clear_figure()
        self.temp_datum.clear_data()
        self.temp_datum.clear_figure()
        if reset_data: self.clear_data()

    def clear_data(self):
        self.center_count = 0
        self.distance = 0
        self.phase = 0.
        self.NR_counter = 0
        self.thread_counter = 0
        self._pid_trial_start_time = 0 

    def thread_reader(self, appendPos=False, appendVel=False):
        while not self.temp_datum.flag_close_event:
            self.arduino.read_single(prt=False, in_waiting=True)
            if self.arduino.receive.rstrip() == "Kill switch hit.":
                self.temp_datum.flag_close_event = True
                print("Kill switch hit detected by thread reader.")
                break
            try:
                self.df.update_data(self.arduino.receive.rstrip().split(','), appendPos=appendPos, appendVel=appendVel)
                self.data.append_data(self.df, appendPos=appendPos, appendVel=appendVel)
            except (ValueError, IndexError):
                if hasattr(self.arduino, 'board'):
                    self.arduino.board.reset_input_buffer()
                pass
    
    def center(self):
        """Now waits for centering confirmation from the Arduino."""
        self.module_name = r"center"
        print("Waiting for centering confirmation from Arduino...")
        centering_successful = False
        start_center_time = time.time()
        while time.time() - start_center_time < 20:
            self.arduino.read_all()
            msg = self.arduino.receive.rstrip()
            if ',' in msg and msg.split(',')[0].isdigit():
                self.center_count, self.distance = int(msg.split(',')[0]), int(msg.split(',')[1])
                print(f"Centering successful. Rail Distance: {self.distance}")
                centering_successful = True
                break
            time.sleep(0.2)
        
        if not centering_successful:
            print("FATAL: Centering failed or timed out. Ending program.")
            if hasattr(self.arduino, 'board') and self.arduino.board.is_open:
                self.arduino.board.close()
            exit()

    def pid(self):
        self.module_name = r"pid"
        try:
            self.data.path = self.path + r"\pid_set_" + str(self._current_pid_param_index + 1)
            os.makedirs(self.data.path, exist_ok=True)
        except OSError:
            pass

        if self.flag_list["pid_input"]:
            pid_params_to_send = PID_PARAM_SETS[self._current_pid_param_index]
            self.arduino.send_message(pid_params_to_send + '\n')
            print(f"Sent PID parameters: {pid_params_to_send}")
            self.data.pid_param = pid_params_to_send
            
            start_ack_time = time.time()
            ack_received = False
            while time.time() - start_ack_time < 10:
                self.arduino.read_all()
                ack_message = self.arduino.receive.rstrip()
                if ack_message and "Start inversion control." in ack_message:
                    print("Arduino acknowledged PID start. Moving to data collection.")
                    ack_received = True
                    break
                time.sleep(0.1)
            
            if ack_received:
                self.flag_list["pid_input"] = False
                self._pid_trial_start_time = time.time()
            else:
                print("Timeout: Did not receive 'Start inversion control.' acknowledgment. Ending trial.")
                self.temp_datum.flag_close_event = True

        else: # Main trial loop
            if self._pid_trial_start_time > 0 and time.time() - self._pid_trial_start_time > TRIAL_DURATION_SECONDS:
                print(f"TRIAL TIMEOUT: Trial exceeded {TRIAL_DURATION_SECONDS} seconds. Terminating.")
                self.temp_datum.flag_close_event = True

            if self.flag_list["thread_init"]:
                reader = threading.Thread(target=self.thread_reader, args=(True, True))
                reader.start()
                self.flag_list["thread_init"] = False

            if not self.temp_datum.flag_close_event:
                self.temp_datum.copy(self.data)
                self.temp_datum.init_plot(self.module_name)
                self.temp_datum.real_time_plot(self.module_name)
            else:
                print(f"PID Trial {self._current_pid_param_index + 1} finished.")
                self.reconnect(exp=True, send_terminate=True)
                self.flag_list["pid"] = False

    def create_folder(self):
        self.cwd = os.getcwd()
        self.path = self.cwd + r"\cart_pendulum_data"
        try: os.mkdir(self.path)
        except OSError: pass

    def main(self):
        self.create_folder()
        
        work_period_start_time = time.time()
        TEN_MINUTES = 20 * 60
        TWO_MINUTES = 5 * 60

        for index, pid_set in enumerate(PID_PARAM_SETS):
            
            if time.time() - work_period_start_time > TEN_MINUTES:
                print(f"\n{'='*25}\n🔋 Over 20 minutes elapsed. Resting for 5 minutes to cool electromagnet.\n{'='*25}")
                
                if hasattr(self.arduino, 'board') and self.arduino.board.is_open:
                    print("Disconnecting Arduino for cooldown...")
                    self.arduino.send_message("Terminate\n")
                    time.sleep(0.2)
                    self.arduino.board.close()
                    print("Arduino connection closed.")

                time.sleep(TWO_MINUTES)
                
                print("\n✅ Rest complete. Resuming automated tests.")
                work_period_start_time = time.time()

            self._current_pid_param_index = index
            print(f"\n{'='*20}\n--- Starting Automated Test Cycle {index + 1}/{len(PID_PARAM_SETS)} ---\n{'='*20}")
            
            print("Connecting to Arduino...")
            if not (hasattr(self.arduino, 'board') and self.arduino.board.is_open):
                self.arduino.initiate()
                time.sleep(2)
            self.arduino.clear()
            
            self.reset(reset_data=True)
            
            print("Sending command '1' to center the cart...")
            self.arduino.send_message("1\n")
            self.center()
            time.sleep(1)

            print("Sending command '4' for PID control...")
            self.arduino.send_message("4\n")
            
            pid_prompt_key_phrase = "Before press ENTER, make sure the pendulum is stable"
            pid_prompt_received = False
            start_wait_time = time.time()
            while time.time() - start_wait_time < 15:
                self.arduino.read_all()
                if pid_prompt_key_phrase in self.arduino.receive:
                    print("PID parameter prompt received. Sending ENTER to proceed...")
                    self.arduino.send_message("\n")
                    time.sleep(0.2)
                    pid_prompt_received = True
                    break
                time.sleep(0.1)
            
            if not pid_prompt_received:
                print(f"FAILED to receive PID prompt for trial {index + 1}. Skipping to next trial.")
                self.reconnect(send_terminate=True)
                continue

            self.flag_list["pid"] = True
            self.flag_list["pid_input"] = True
            self.flag_list["thread_init"] = True
            
            while self.flag_list["pid"]:
                try:
                    self.pid()
                except KeyboardInterrupt:
                    print("\nKeyboardInterrupt during PID run. Terminating trial.")
                    self.temp_datum.flag_close_event = True
            
            print(f"--- Test Cycle {index + 1} Complete ---")
            print("Waiting 8 seconds before starting the next trial...")
            time.sleep(8)
        
        print("\n✅ All automated PID tests are finished.")

if __name__ == "__main__":
    arduino_board = arduino(port, baudrate)
    df = data_frame()
    datum = data(fft_length=512, sampling_div=0.04, wait_to_stable=1)
    temp_datum = live_data(fft_length=512, sampling_div=0.04, wait_to_stable=1)
    cartER = cart_pendulum(arduino_board, datum, temp_datum, df)

    cartER.main()
    
    print("\nProgram ends.")