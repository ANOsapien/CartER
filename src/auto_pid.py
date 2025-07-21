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
MAX_COUNT = 10 # Number of points waited to plot a frame
ANGLE_ROTATION = 55 # Rotation of the y-label

# Define a list of PID parameter sets for automated testing
# Each string represents Kp_ang,Ki_ang,Kd_ang,Kp_pos,Ki_pos,Kd_pos
PID_PARAM_SETS = [
    "600,400,2.5,-0.05,0,-0.01",   # Set 1 (Original)
    "700,450,3.0,-0.06,0,-0.015",  # Set 2
    "550,350,2.0,-0.04,0,-0.008",  # Set 3
    "650,420,2.8,-0.055,0,-0.012", # Set 4
    "580,380,2.3,-0.045,0,-0.009", # Set 5
    "720,480,3.2,-0.07,0,-0.018",  # Set 6
    "500,300,1.8,-0.03,0,-0.005",  # Set 7
    "800,500,3.5,-0.08,0,-0.02",   # Set 8
    "620,410,2.6,-0.052,0,-0.011", # Set 9
    "750,470,3.1,-0.065,0,-0.016"  # Set 10
]

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
        # A dictionary of flags to control the system
        self.flag_list = {
            "command": True, # whether a command is sent to the arduino
            "reset": False, # reset command
            "center": False, # center command
            "pid": False, # pid command
            "measure": False, # measure command
            "NR": False, # Normalised Resonance command
            "setSpeed": False, # set speed and acceleration command
            "freq_scan": False, # frequency scan command
            "multi_freq": False, # whether multiple frequencies are sent
            "omega": True, # Input driven frequency command
            "amp": True, # Input varying amplitude command
            "amp_0": True, # Input constant amplitude command
            "swing_request": True, # whether the swing is requested
            "pid_input": True, # whether the pid input is requested
            "thread_init": True, # whether the thread is initiated
            "flag_scan": True, # whether run scanning mode
            "setSpeed_request": True, # whether the setSpeed is requested
        }
        self.init_true_flag_list = ["command",
                                     "omega",
                                     "amp",
                                     "amp_0",
                                     "swing_request",
                                     "pid_input",
                                     "thread_init",
                                     "flag_scan",
                                     "setSpeed_request"]
        self.init_false_flag_list = ["reset",
                                     "center",
                                     "pid",
                                     "measure",
                                     "NR",
                                     "setSpeed",
                                     "freq_scan",
                                     "multi_freq",]
        self.reset_dict = { # To renew the flag_command
            "Resetting...",
            "No command detected.",
            "Unidentified command. Please try again.",
            "More than one command detected. Resetting the values.",
            "Hasn't been centred. Please centre the cart first.",
            "Terminating since limit switch pressed...",
        }
        self.command_dict = {   # these must match the messages from the Arduino's command_print() exactly!
            "Beginning centring.": "center",
            "Beginning PID control.": "pid",
            "Beginning measuring the natural frequency and quality factor.": "measure",
            "Beginning the normalised resonance.": "NR",
            "Beginning setting the speed and acceleration.": "setSpeed",
            "Beginning the frequency scan.": "freq_scan",
        }
        # Flags for automated testing loop management
        self._auto_restart = False
        self._should_restart_automation = False
        self._current_pid_param_index = 0

    def reset_flag_list(self, swing_request = False):
        for flag in self.init_true_flag_list:
            self.flag_list[flag] = True
        for flag in self.init_false_flag_list:
            self.flag_list[flag] = False
        if(swing_request):
            self.flag_list["swing_request"] = False

    def reconnect(self,
                  exp = False,
                  swing_request = False,
                  send_terminate = False,
                  NR_phase_amp = False,
                  manual_continue = True,
                  input_spec_info = True,
                  ):
        '''This function stops the serial connection and waits for ENTER to reconnect'''
        if(send_terminate):
            time.sleep(0.1)
            self.arduino.send_message("Terminate\n")
        try:
            plt.close("all")
            # Close the current serial connection if it's open
            if self.arduino.board and self.arduino.board.is_open:
                self.arduino.read_single() # Clear any last message
                self.arduino.clear() # Clear buffer
                self.arduino.board.close()
                print("Arduino connection closed for reconnect.")
            time.sleep(0.1)
            if(exp):
                self.temp_datum.export_csv(self.module_name,
                                           NR_phase_amp = NR_phase_amp,
                                           input_spec_info = input_spec_info,)
            if(manual_continue):
                print("\nPress ENTER to reconnect.\n\nOr press CTRL+C then ENTER to exit the program.\n")
                if self._auto_restart:
                    print("Auto-restarting triggered by reconnect...")
                    self._should_restart_automation = True # Signal main loop to restart automation
                    # DO NOT call arduino.initiate() or self.reset() here in auto-restart mode
                    # The main loop will handle the full restart sequence.
                else:
                    input() # Original behavior: wait for user
                    self.arduino.initiate() # Manual reconnect
                    self.reset(reset_data = False, swing_request = swing_request) # Manual reset
        except KeyboardInterrupt:
            if self.arduino.board and self.arduino.board.is_open:
                self.arduino.board.close()
            print("KeyboardInterrupt during reconnect. Exiting.")
            exit() # Exit the program cleanly
        
        if not manual_continue and not self._auto_restart:
            self.reset(reset_data = False, swing_request = swing_request)

    def reset(self, reset_data = True, swing_request = False):
        '''Resets Python's internal state and flags. Connection re-initiation is handled externally.'''
        self.arduino.clear() # Clear Arduino's buffer

        # Reset Python flags
        self.reset_flag_list(swing_request = swing_request)

        # Clear Python data structures
        self.data.clear_data()
        self.data.clear_figure()
        self.temp_datum.clear_data()
        self.temp_datum.clear_figure()
        if(reset_data):
            self.clear_data()

    def clear_data(self):
        self.center_count = 0
        self.distance = 0
        self.phase = 0.
        self.NR_counter = 0
        self.thread_counter = 0

    def command_flag(self):
        # command flag controlled by the arduino output
        result = self.arduino.receive.rstrip()
        while (result.startswith("DEBUG")):  # should have already been handled by arduino.read_single()
            self.arduino.read_single()
            result = self.arduino.receive.rstrip()
        if(result not in self.reset_dict \
            and result in self.command_dict):
            self.flag_list[self.command_dict[result]] = True
            self.flag_list["command"] = False
        elif (result in self.reset_dict):
            # If Arduino sends a reset message, explicitly trigger Python's reset
            print(f"Arduino sent a reset message: '{result}'. Setting Python reset flag.")
            self.flag_list["command"] = False # Temporarily unset command to handle reset
            self.flag_list["reset"] = True # Set reset flag to be handled in main loop
            # The main loop will call self.reset(), which will then re-initiate connection
        else:
            self.flag_list["command"] = True

    def command(self):
        self.arduino.read_all()
        self.arduino.send_command()
        self.arduino.read_all() # Read response to command
        self.command_flag()

    def thread_reader(self,
                      appendPos = False,
                      appendVel = False,
                      thread_check = False):
        while(not self.temp_datum.flag_close_event):
            self.arduino.read_single(prt = False, in_waiting = True)
            if(self.arduino.receive.rstrip() == "Kill switch hit."):
                self.temp_datum.flag_close_event = True
                print("Kill switch hit detected by thread reader.")
                break
            try:
                self.df.update_data(self.arduino.receive.rstrip().split(','), \
                    appendPos = appendPos, appendVel = appendVel)
                self.data.append_data(self.df, appendPos = appendPos, appendVel = appendVel)
                if(thread_check):
                    print("time_sys: %.3f time_read: %.3f thread_counter: %d" % \
                        ((time.time() - self.data.sys_start_time), (self.df.time - self.data.start_time), self.thread_counter))
                    self.thread_counter += 1
            except ValueError:
                self.arduino.board.reset_input_buffer()
                pass

    def thread_writer(self):
        while(not self.temp_datum.flag_close_event):
            msg = input("Send the new amplitude/steps (Press Ctrl+C to exit!!!)\n") + "\n"
            try:
                a = float(msg.split(',')[0])
                if(self.data.omega * abs(a) > 2000):
                    print("The amplitude is too large. Please try again.\n")
                else:
                    msg = str(abs(a)) + "," + str(self.phase) + "\n"
                    self.arduino.send_message(msg)
                    self.temp_datum.amp = abs(a)
                    self.data.amp = abs(a)
                    print("sent amp, phase: " + msg)
                    # BUG: not sending the phase at the same time!
            except ValueError:
                print("Invalid input, please try again.\n")
            time.sleep(2) # wait 2 seconds for the transient behaviour to fade away a bit

    def center(self):
        self.module_name = r"center"
        self.arduino.read_single(prt = False)
        self.center_count, self.distance = int(self.arduino.receive.rstrip().split(',')[0]),\
            int(self.arduino.receive.rstrip().split(',')[1])
        print("Centering done: ", self.center_count, "\tRail Distance: ", self.distance, "\n")
        self.reset_flag_list()

    def measure(self):
        self.module_name = r"measure"
        try:
            self.data.path = self.path + r"\measure"
            os.makedirs(self.data.path)
        except OSError:
            pass
        if(self.flag_list["thread_init"]):
            reader = threading.Thread(target = self.thread_reader,
                                      args = (False, False, False))
            reader.start()
            self.flag_list["thread_init"] = False
        # plot the graph in the main thread
        if(not self.temp_datum.flag_close_event):
            self.temp_datum.copy(self.data)
            self.temp_datum.init_plot(self.module_name)
            self.temp_datum.real_time_plot(self.module_name)
        else:
            self.reconnect(exp = True)

    def setSpeed(self):
        '''Set the speed and acceleration of the cart'''
        self.module_name = r"setSpeed"
        try:
            self.data.path = self.path + r"\setSpeed"
            os.makedirs(self.data.path)
        except OSError:
            pass
        if(self.flag_list["setSpeed_request"]):
            self.arduino.read_all()
            self.arduino.send_input_message(save_to_omega = False)
            self.arduino.read_single()
            if(self.arduino.receive.rstrip().startswith("Start sinusoidal motion with")):
                self.flag_list["setSpeed_request"] = False
                self.data.setSpeed_param = self.arduino.receive.rstrip().replace("Start sinusoidal motion with ", "")
        else:
            if(self.flag_list["amp_0"]):
                self.arduino.read_all()
                self.arduino.send_input_message(save_to_omega = False)
                msg_amp = self.arduino.message.rstrip()
                self.arduino.read_single()
                if(self.arduino.receive.rstrip().startswith("Starting with amplitude:")):
                    self.flag_list["amp_0"] = False
                    self.data.amp_0 = float(msg_amp)
                    self.temp_datum.amp_0 = float(msg_amp)
            else:
                if(self.arduino.receive.rstrip() == "Kill switch hit."):
                    print("Kill switch hit. Resetting the system...\n")
                    self.reconnect(exp = True)
                else:
                    if(self.flag_list["thread_init"]):
                        reader = threading.Thread(target = self.thread_reader,
                                                  args = (True, True, False))
                        reader.start()
                        self.flag_list["thread_init"] = False

                    if(not self.temp_datum.flag_close_event):
                        self.temp_datum.copy(self.data)
                        self.temp_datum.init_plot(self.module_name)
                        self.temp_datum.real_time_plot(self.module_name)
                    else:
                        self.reconnect(exp = True)
    
    def pid(self):
        self.module_name = r"pid"
        try:
            self.data.path = self.path + r"\pid"
            os.makedirs(self.data.path)
        except OSError:
            pass

        if False and self.flag_list["swing_request"]: # This block appears to be intentionally bypassed
            print("swing_request")
            self.arduino.read_single()
            time.sleep(0.1)
            self.arduino.send_message('n' + '\n')
            self.arduino.read_single()
            if self.arduino.receive.rstrip() in [
                "Continue with swing-up strategy.",
                "Continue without swing-up strategy."
            ]:
                self.flag_list["swing_request"] = False

        else:
            # If this flag is true, it means main() has already received the prompt.
            if self.flag_list["pid_input"]:
                print("pid_input phase triggered.")
                
                # DO NOT re-read the buffer. The prompt was already found.
                # Just send the parameters immediately.
                print("Arduino is ready for PID parameters. Sending now...")
    
                # Get the current PID parameter set from the global list
                pid_params_to_send = PID_PARAM_SETS[self._current_pid_param_index]
                self.arduino.send_message(pid_params_to_send + '\n')
                print(f"Sent PID parameters: {pid_params_to_send}")
                self.data.pid_param = pid_params_to_send # Store the sent parameters
    
                # Now, proceed with the original logic to wait for the *start acknowledgment*
                start_ack_time = time.time()
                max_ack_wait_time = 5
                ack_received = False
                while time.time() - start_ack_time < max_ack_wait_time:
                    time.sleep(0.1)
                    self.arduino.read_all()
                    ack_message = self.arduino.receive.rstrip()
                    if ack_message:
                        print(f"Waiting for PID start ack... Received: {ack_message}")
                        if "Start inversion control." in ack_message:
                            print("Arduino acknowledged PID start. Moving to data collection.")
                            ack_received = True
                            break
                
                if ack_received:
                    self.flag_list["pid_input"] = False
                else:
                    print("Timeout: Did not receive 'Start inversion control.' acknowledgment.")
                    # Keep pid_input as True to allow for potential retries or error handling

            else: # pid_input is False, meaning parameters have been sent and control has started
                if self.arduino.receive.rstrip() == "Kill switch hit.":
                    print("Kill switch hit. Resetting the system...\n")
                    self.reconnect(exp=True, send_terminate=True) # Send terminate to Arduino
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
                        self.reconnect(exp=True) # This reconnect will be handled for auto-restart

    def create_folder(self):
        self.cwd = os.getcwd()
        self.path = self.cwd + r"\cart_pendulum_data"
        try:
            os.mkdir(self.path)
        except OSError:
            pass

    def main(self, automate_setup=False, auto_restart=False):
        self.create_folder()
        self._auto_restart = auto_restart # Set the auto_restart flag for the instance

        # Initial connection outside the main loop to ensure it's established once
        self.arduino.initiate()
        self.arduino.clear() # Clear any initial garbage after connection

        # Flag to control the initial automated setup for each PID set
        self._should_restart_automation = automate_setup

        while True: # Outer loop for continuous operation of PID tests
            if self._should_restart_automation:
                print(f"\n--- Starting Automated Test Cycle {self._current_pid_param_index + 1}/{len(PID_PARAM_SETS)} ---")

                if self._current_pid_param_index >= len(PID_PARAM_SETS):
                    print("All PID parameter sets tested. Program ending.")
                    break # Exit the main loop

                # Ensure Arduino connection is open before starting a new cycle
                if not (self.arduino.board and self.arduino.board.is_open):
                    print("Arduino connection not open. Attempting to re-initiate...")
                    self.arduino.initiate()
                    time.sleep(1) # Give it a moment to stabilize
                    self.arduino.clear() # Clear buffer after re-initiate

                # Reset Python flags and data for a fresh start
                self.reset(reset_data=True)

                # Auto-center
                print("Sending '1' to center...")
                self.arduino.send_message("1\n")
                centering_successful = False
                for _ in range(50): # Wait up to 10 seconds (50 * 0.2s) for centering
                    self.arduino.read_all()
                    msg = self.arduino.receive.rstrip()
                    if ',' in msg and msg.split(',')[0].isdigit():
                        print(f"Centering done. Message: {msg}")
                        centering_successful = True
                        break
                    time.sleep(0.2)
                
                if not centering_successful:
                    print("Centering failed. Exiting program.")
                    self.arduino.board.close()
                    break # Exit the while True loop

                print("Centered! Starting PID trials in 5 seconds...\n")
                time.sleep(5)

                # Auto-send PID command '4' and WAIT for the parameter prompt.
                print("Sending '4' for PID control and waiting for parameter prompt...")
                self.arduino.send_message("4\n")
                
                # Loop to read until the specific PID parameter prompt is received
                pid_prompt_key_phrase_1 = "Before press ENTER, make sure the pendulum is stable at either the down or upright position!"
                pid_prompt_key_phrase_2 = "Resume (ENTER r) or ENTER six numbers split by commas without spaces"
                
                pid_prompt_received = False
                max_wait_time = 15 # Max 15 seconds to wait for the prompt (increased for robustness)
                start_wait_time = time.time()
                full_arduino_response_buffer = "" # Accumulate all received messages

                while time.time() - start_wait_time < max_wait_time:
                    time.sleep(0.1) # Small delay to allow Arduino to send full message
                    self.arduino.read_all()
                    current_chunk = self.arduino.receive.rstrip()
                    if current_chunk: # Append non-empty chunks
                        full_arduino_response_buffer += current_chunk + "\n" # Add newline to separate logical messages
                    
                    # Check if either key phrase is in the accumulated response
                    if pid_prompt_key_phrase_1 in full_arduino_response_buffer or \
                       pid_prompt_key_phrase_2 in full_arduino_response_buffer:
                        print(f"PID parameter prompt received (from Arduino after sending '4'):\n---ARDUINO RESPONSE START---\n{full_arduino_response_buffer.strip()}\n---ARDUINO RESPONSE END---")
                        pid_prompt_received = True
                        break
                    
                    elif "Beginning PID control." in current_chunk and not pid_prompt_received:
                        print(f"Received 'Beginning PID control.' but waiting for full prompt...")

                if not pid_prompt_received:
                    print("Failed to receive PID parameter prompt from Arduino after sending '4'. Exiting program.")
                    print(f"Last full buffer received: \n{full_arduino_response_buffer.strip()}")
                    self.arduino.board.close()
                    break # Exit the while True loop

                # Set flags for PID to proceed automatically
                self.flag_list["command"] = False # No longer in command selection phase
                self.flag_list["pid"] = True      # Now in PID mode
                self.flag_list["pid_input"] = True # We are in the phase to input PID params
                self.flag_list["thread_init"] = True # Ensure thread is initialized for data reading
                self._should_restart_automation = False # Automation setup done for this cycle

            # Main loop logic to process current flags
            try:
                if(self.flag_list["command"]):
                    try:
                        self.command()
                    except KeyboardInterrupt:
                        self.reconnect()
                elif(self.flag_list["reset"]):
                    print("Python detected 'reset' flag. Calling self.reset()...")
                    self.reset(reset_data=True) # Reset Python state
                    self._should_restart_automation = True # Signal for next automated test
                    self.flag_list["reset"] = False # Clear the reset flag
                elif(self.flag_list["center"]):
                    try:
                        self.center()
                    except KeyboardInterrupt:
                        self.reconnect()
                elif(self.flag_list["pid"]):
                    try:
                        self.pid()
                    except KeyboardInterrupt:
                        print("KeyboardInterrupt during PID run. Terminating Arduino and restarting cycle.")
                        self.reconnect(exp = True, send_terminate = True)
                        self._should_restart_automation = True
                elif(self.flag_list["measure"]):
                    try:
                        self.measure()
                    except KeyboardInterrupt:
                        self.reconnect(exp = True)
                elif(self.flag_list["NR"]):
                    try:
                        self.NR(NR_scan = False, interpolation = True)
                    except KeyboardInterrupt:
                        self.reconnect(exp = True, send_terminate = True, NR_phase_amp = False)
                elif(self.flag_list["setSpeed"]):
                    try:
                        self.setSpeed()
                    except KeyboardInterrupt:
                        self.reconnect(exp = True, send_terminate = True)
                elif(self.flag_list["freq_scan"]):
                    try:
                        self.freq_scan()
                    except KeyboardInterrupt:
                        self.reconnect(exp = True, send_terminate = True)
            except KeyboardInterrupt:
                print("\nMain loop interrupted by KeyboardInterrupt. Closing Arduino and exiting.")
                if self.arduino.board and self.arduino.board.is_open:
                    self.arduino.board.close()
                self.reset_flag_list() # Clean up flags
                break # Exit the while True loop


if(__name__ == "__main__"):
    # Initiation parameters
    fft_lengths = 512
    sampling_divs = 0.04
    wait_to_stables = 1

    arduino_board = arduino(port, baudrate)
    df = data_frame()
    datum = data(fft_length = fft_lengths,
                 sampling_div = sampling_divs,
                 wait_to_stable = wait_to_stables)
    temp_datum = live_data(fft_length = fft_lengths,
                           sampling_div = sampling_divs,
                           wait_to_stable = wait_to_stables)
    cartER = cart_pendulum(arduino_board, datum, temp_datum, df)

    cartER.main(automate_setup=True, auto_restart=True)
    print("\nProgram ends.")