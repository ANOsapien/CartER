import os
import time
import random
import threading
import numpy as np
import matplotlib as mpl, matplotlib.pyplot as plt
# import modules from other python files
from data_process import data, live_data
from arduino_manager import arduino
from moment_data_process import data_frame
from deap import base, creator, tools, algorithms
import pandas as pd

# Matplotlib and plotting setup
plt.rcParams['axes.grid'] = True
plt.rcParams['figure.autolayout'] = True
prop_cycle = plt.rcParams['axes.prop_cycle']
colors = prop_cycle.by_key()['color']
mpl.use('TkAgg')

# Serial port settings
port = 'COM4'
baudrate = 230400

# GA & PID utility functions ----------------------------------------
def load_trial_csv(path):
    """
    Parse a CSV exported by export_csv: special_info then time-series block.
    """
    with open(path) as f:
        lines = [l.strip() for l in f if l.strip()]
    # find time-series header
    for i, line in enumerate(lines):
        if line.startswith("time,"):
            header_idx = i
            break
    # parse special info
    info = {}
    for ln in lines[:header_idx]:
        k, *vals = ln.split(',')
        info[k] = float(vals[0]) if len(vals)==1 else vals
    # build dataframe
    ts = "\n".join(lines[header_idx:])
    df = pd.read_csv(pd.compat.StringIO(ts))
    return info, df


def compute_metrics(df, setpoint=0.0):
    """
    Compute ISE, overshoot, and settling time from position data.
    """
    t = df['time'].values
    y = df['position'].values
    e = y - setpoint
    ISE = np.trapz(e**2, t)
    final = y[-1]
    overshoot = np.max(y) - final
    tol = 0.05 * abs(final)
    idx = np.where(np.abs(e) < tol)[0]
    st = t[idx[0]] if len(idx)>0 else t[-1]
    return ISE, overshoot, st


def fitness_from_metrics(ISE, overshoot, settling_time,
                         w1=1.0, w2=10.0, w3=1.0):
    """
    Single-objective: higher is better (negated cost).
    """
    return -(w1*ISE + w2*overshoot + w3*settling_time)

# DEAP GA setup -----------------------------------------------------
creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)
toolbox = base.Toolbox()
# initial user parameters for GA ranges
INIT = [600, 400, 2.5, -0.05, 0, -0.01]

def attr_float(idx):
    return random.uniform(0.5*INIT[idx], 1.5*INIT[idx])
# register six attributes
for i in range(6): toolbox.register(f"attr_{i}", lambda idx=i: attr_float(idx))
toolbox.register("individual", tools.initCycle, creator.Individual,
                 [toolbox.attr_0, toolbox.attr_1, toolbox.attr_2,
                  toolbox.attr_3, toolbox.attr_4, toolbox.attr_5], n=1)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
toolbox.register("mate", tools.cxBlend, alpha=0.5)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.2, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)

def default_evaluate(ind):
    # placeholder, overwritten at runtime
    return (0.0,)

toolbox.register("evaluate", default_evaluate)

# Main cart_pendulum class -----------------------------------------
class cart_pendulum:
    '''Cart pendulum manager with GA PID tuning.'''    
    def __init__(self, arduino, data, temp_data, data_frame):
        self.arduino = arduino
        self.data = data
        self.temp_datum = temp_data
        self.df = data_frame
        self.module_name = r"\Default_Cart_Pendulum"
        self.path = os.path.join(os.getcwd(), 'cart_pendulum_data')
        self.sent_params = []  # history of GA outputs

        # copy your existing flag initialization
        self.flag_list = {
            "command": True, "reset": False, "center": False,
            "pid": False, "measure": False, "NR": False,
            "setSpeed": False, "freq_scan": False, "multi_freq": False,
            "omega": True, "amp": True, "amp_0": True,
            "swing_request": True, "pid_input": True,
            "thread_init": True, "flag_scan": True,
            "setSpeed_request": True
        }
        self.init_true_flag_list = ["command","omega","amp","amp_0",
                                    "swing_request","pid_input",
                                    "thread_init","flag_scan",
                                    "setSpeed_request"]
        self.init_false_flag_list = ["reset","center","pid",
                                     "measure","NR","setSpeed",
                                     "freq_scan","multi_freq"]
        self.reset_dict = {
            "Resetting...","No command detected.",
            "Unidentified command. Please try again.",
            "More than one command detected. Resetting the values.",
            "Hasn't been centred. Please centre the cart first.",
            "Terminating since limit switch pressed..."
        }
        self.command_dict = {
            "Beginning centring.": "center",
            "Beginning PID control.": "pid",
            "Beginning measuring the natural frequency and quality factor.": "measure",
            "Beginning the normalised resonance.": "NR",
            "Beginning setting the speed and acceleration.": "setSpeed",
            "Beginning the frequency scan.": "freq_scan"
        }

    def find_latest_csv_dir(self):
        pid_parent = os.path.join(self.path, 'pid')
        subs = [os.path.join(pid_parent,d) for d in os.listdir(pid_parent)
                if os.path.isdir(os.path.join(pid_parent,d))]
        return max(subs, key=os.path.getmtime)

    def run_ga(self, csv_dir, ngen=20, pop_size=10):
        # collect all CSVs
        files = [os.path.join(csv_dir,f) for f in os.listdir(csv_dir)
                 if f.lower().endswith('.csv')]
        # define multi-file fitness
        def eval_multi(ind):
            total = 0.0
            for path in files:
                _, df = load_trial_csv(path)
                ISE, ov, st = compute_metrics(df)
                total += fitness_from_metrics(ISE, ov, st)
            return (total,)
        toolbox.unregister("evaluate")
        toolbox.register("evaluate", eval_multi)
        pop = toolbox.population(n=pop_size)
        hof = tools.HallOfFame(1)
        algorithms.eaSimple(pop, toolbox, cxpb=0.5, mutpb=0.2,
                            ngen=ngen, halloffame=hof, verbose=False)
        return hof[0]

    def pid(self):
        """
        Automate PID entry: on prompt from Arduino, run GA over latest CSVs
        and send best parameters.
        """
        self.module_name = r"pid"
        # ensure pid folder exists
        try:
            self.data.path = os.path.join(self.path, "pid")
            os.makedirs(self.data.path, exist_ok=True)
        except OSError:
            pass

        if self.flag_list["pid_input"]:
            print("pid_input: waiting for Arduino prompt...")
            # read until the stability prompt appears
            received = ""
            # keep reading until prompt line seen
            while True:
                self.arduino.read_single()
                received += self.arduino.receive
                if "Before press ENTER, make sure the pendulum is stable" in self.arduino.receive:
                    break
                time.sleep(0.1)
            print("Stability prompt received, running GA tuner...")
            # locate the latest CSV directory
            latest = self.find_latest_csv_dir()
            print(f"Running GA on CSV folder: {latest}")
            best = self.run_ga(latest, ngen=30, pop_size=15)
            pid_params = ','.join(f"{v:.4f}" for v in best)
            self.sent_params.append(pid_params)
            print(f"Sending PID params: {pid_params}")
            # send parameters to Arduino
            self.arduino.send_message(pid_params + '')
            self.data.pid_param = pid_params
            self.flag_list["pid_input"] = False

            # read acknowledgement
            time.sleep(0.5)
            self.arduino.read_all()
            print(self.arduino.receive)
            if "Start inversion control." in self.arduino.receive:
                print("Arduino acknowledged PID start.")
        else:
            # follow original post-entry handling
            if self.arduino.receive.rstrip() == "Kill switch hit.":
                print("Kill switch hit. Resetting the system...")
                self.reconnect(exp=True)
            else:
                if self.flag_list["thread_init"]:
                    reader = threading.Thread(
                        target=self.thread_reader,
                        args=(True, True, False)
                    )
                    reader.start()
                    self.flag_list["thread_init"] = False

                if not self.temp_datum.flag_close_event:
                    self.temp_datum.copy(self.data)
                    self.temp_datum.init_plot(self.module_name)
                    self.temp_datum.real_time_plot(self.module_name)
                else:
                    self.reconnect(exp=True)

    # include all your other methods unchanged below this marker
    # ...command, reset, center, measure,
    # setSpeed, thread_reader, thread_writer, reconnect, etc.) unchanged
    # ...

    def main(self):
        input("\nPress ENTER to begin connection...\n")
        # ensure data folder exists
        os.makedirs(self.path, exist_ok=True)
        self.arduino.initiate()
        while self.arduino.board.is_open:
            try:
                if self.flag_list["command"]:
                    self.command()
                elif self.flag_list["reset"]:
                    self.reset(reset_data=True)
                elif self.flag_list["center"]:
                    self.center()
                elif self.flag_list["pid"]:
                    self.pid()
                elif self.flag_list["measure"]:
                    self.measure()
                elif self.flag_list["NR"]:
                    self.NR(NR_scan=False, interpolation=True)
                elif self.flag_list["setSpeed"]:
                    self.setSpeed()
                elif self.flag_list["freq_scan"]:
                    self.freq_scan()
            except KeyboardInterrupt:
                self.arduino.board.close()
                # reset flags
                for f in self.init_true_flag_list:
                    self.flag_list[f] = True
                for f in self.init_false_flag_list:
                    self.flag_list[f] = False
                break

if __name__ == "__main__":
    # init classes
    arduino_board = arduino(port, baudrate)
    df = data_frame()
    datum = data(fft_length=512, sampling_div=0.04, wait_to_stable=1)
    temp_datum = live_data(fft_length=512, sampling_div=0.04, wait_to_stable=1)
    cartER = cart_pendulum(arduino_board, datum, temp_datum, df)

    # run 10 trials automatically
    cartER.arduino.initiate()
    for i in range(10):
        print(f"\n--- Trial {i+1}/10 ---")
        cartER.arduino.send_message("\n")  # center

        cartER.arduino.send_message("1\n")  # center
        time.sleep(1)
        cartER.arduino.read_all()
        cartER.arduino.send_message("4\n")  # enter PID
        time.sleep(0.5)
        cartER.arduino.read_all()
        cartER.pid()
    print("All 10 trials completed.")
