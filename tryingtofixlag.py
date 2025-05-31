import socket
import threading
import sys
import os
import json
import time
from pathlib import Path
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.clock import Clock
from kivy.config import Config

# Import the inputs library
from inputs import get_gamepad, UnpluggedError, devices

# UDP Configuration
UDP_IP = "192.168.5.60"
UDP_PORT_SEND = 44158
UDP_PORT_RECEIVE = 44159

# Declare global variables
global x_axis_left, x_axis_right, y_axis_left, y_axis_right, loop_state, virtualloop_enabled, preset_a_slide, preset_b_slide, trigger_left_mapped, trigger_right_mapped, ms_speed, invert_left, invert_right, invert_ry
global initial_setup_complete, joystick_status_label, error_label, joystick_input_label, joystick_thread, gamepad, gamepad_initialized
global last_save_time, json_lock, sensitivity_right, sensitivity_label

x_axis_left = 0.0
x_axis_right = 0.0
y_axis_left = 0.0
y_axis_right = 0.0
trigger_left_mapped = 0.0
trigger_right_mapped = 0.0
ms_speed = 800
invert_left = False
invert_right = False
invert_ry = False
initial_setup_complete = False
joystick_thread = None
gamepad = None
gamepad_initialized = False
last_save_time = 0
SAVE_INTERVAL = 5.0  # Save JSON every 5 seconds
json_lock = threading.Lock()
sensitivity_right = 1.0  # Default sensitivity for right joystick (1.0 = full, 0.1 = minimum)
sensitivity_label = None  # Will be set in the build method

# Initialize UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT_RECEIVE))

# --- Inputs library setup ---
def initialize_gamepad():
    global gamepad, gamepad_initialized
    try:
        gamepads = devices.gamepads
        if gamepads:
            gamepad = gamepads[0]
            print(f"Gamepad detected: {gamepad.name}")
            gamepad_initialized = True
            return True
        else:
            print("No gamepad detected!")
            gamepad_initialized = False
            return False
    except Exception as e:
        print(f"Error initializing gamepad: {e}")
        gamepad_initialized = False
        return False

# Attempt to initialize gamepad at startup
gamepad_initialized = initialize_gamepad()

# Deadzone threshold
DEADZONE = 0.1

# Control states
loop_state = False
virtualloop_enabled = False

# Button state tracking to avoid spamming (using a dictionary for 'inputs' events)
last_button_states = {}

# File path for saving motor parameters
PRIMARY_PATH = Path(os.path.expanduser("~")) / "Documents" / "Controller2" / "motor_params.json"
FALLBACK_PATH = Path(sys.executable).parent / "motor_params.json"
PARAMS_FILE = None

def set_params_file_path():
    global PARAMS_FILE
    try:
        PRIMARY_PATH.parent.mkdir(parents=True, exist_ok=True)
        test_file = PRIMARY_PATH.parent / "test_write_access.tmp"
        with open(test_file, "w") as f:
            f.write("test")
        test_file.unlink()
        PARAMS_FILE = PRIMARY_PATH
        print(f"Using primary path for JSON: {PARAMS_FILE}")
    except (PermissionError, OSError) as e:
        print(f"Cannot use primary path {PRIMARY_PATH}: {e}. Falling back to executable directory.")
        try:
            PARAMS_FILE = FALLBACK_PATH
            PARAMS_FILE.parent.mkdir(parents=True, exist_ok=True)
            print(f"Using fallback path for JSON: {PARAMS_FILE}")
        except Exception as e:
            print(f"Cannot use fallback path {FALLBACK_PATH}: {e}. JSON file operations will fail.")
            PARAMS_FILE = None

# Function to save motor parameters to JSON file
def save_params_to_json():
    global error_label, invert_left, invert_right, invert_ry, sensitivity_right
    if PARAMS_FILE is None:
        error_label.text = "Error: Cannot save JSON (no valid path)"
        print("No valid path for JSON file. Cannot save parameters.")
        return
    with json_lock:
        try:
            params = {
                "current_positions": {
                    "motor1": int(current_pos1_label.text.split(":")[1].strip()),
                    "motor2": int(current_pos2_label.text.split(":")[1].strip()),
                    "motor3": int(current_pos3_label.text.split(":")[1].strip()),
                    "motor4": int(current_pos4_label.text.split(":")[1].strip())
                },
                "preset_a": {
                    "motor1": int(preset_a_label.text.split(":")[1].strip().split(",")[0]),
                    "motor2": int(preset_a_label.text.split(":")[1].strip().split(",")[1]),
                    "motor3": int(preset_a_label.text.split(":")[1].strip().split(",")[2]),
                    "motor4": int(preset_a_label.text.split(":")[1].strip().split(",")[3])
                },
                "preset_b": {
                    "motor1": int(preset_b_label.text.split(":")[1].strip().split(",")[0]),
                    "motor2": int(preset_b_label.text.split(":")[1].strip().split(",")[1]),
                    "motor3": int(preset_b_label.text.split(":")[1].strip().split(",")[2]),
                    "motor4": int(preset_b_label.text.split(":")[1].strip().split(",")[3])
                },
                "ms_speed": ms_speed,
                "udp_ip": UDP_IP,
                "invert_left": invert_left,
                "invert_right": invert_right,
                "invert_ry": invert_ry,
                "sensitivity_right": sensitivity_right
            }
            with open(PARAMS_FILE, "w") as f:
                json.dump(params, f, indent=4)
            print(f"Saved parameters to {PARAMS_FILE}")
            error_label.text = "JSON Saved Successfully"
        except Exception as e:
            error_label.text = f"Error Saving JSON: {str(e)}"
            print(f"Error saving JSON file: {e}")

# Function to load motor parameters from JSON file
def load_params_from_json():
    global error_label, ms_speed, UDP_IP, invert_left, invert_right, invert_ry, sensitivity_right
    if PARAMS_FILE is None:
        error_label.text = "Error: Cannot load JSON (no valid path)"
        print("No valid path for JSON file. Cannot load parameters.")
        return None
    with json_lock:
        if os.path.exists(PARAMS_FILE):
            print(f"Found JSON file at {PARAMS_FILE}")
            try:
                with open(PARAMS_FILE, "r") as f:
                    params = json.load(f)
                invert_left = params.get("invert_left", False)
                invert_right = params.get("invert_right", False)
                invert_ry = params.get("invert_ry", False)
                sensitivity_right = params.get("sensitivity_right", 1.0)
                error_label.text = "JSON Loaded Successfully"
                return params
            except Exception as e:
                error_label.text = f"Error Loading JSON: {str(e)}"
                print(f"Error loading JSON file: {e}")
                return None
        print(f"No JSON file found at {PARAMS_FILE}")
        error_label.text = "No JSON File Found"
        return None

# Function to periodically save parameters
def save_params_periodically(dt):
    global last_save_time
    current_time = time.time()
    if current_time - last_save_time >= SAVE_INTERVAL:
        save_params_to_json()
        last_save_time = current_time

# Function to send UDP message
def send_udp_message(message):
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT_SEND))

# Function to handle received UDP messages
def receive_udp_messages():
    global loop_state, ms_speed, initial_setup_complete
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode()
            print(f"Received from {addr}: {message}")

            if "CURRENT_POS:" in message:
                update_motor_positions(message)
                if not initial_setup_complete:
                    initial_setup_complete = True
                    print("Initial setup complete: Arduino has sent first CURRENT_POS.")
            elif "PRESET_POS_A:" in message and "PRESET_POS_B:" in message:
                update_preset_positions(message)
            elif "MSSPEED:" in message:
                ms_speed = float(message.split("MSSPEED:")[1])
                ms_speed_label.text = f"msSpeed: {ms_speed}"
                save_params_to_json()
            elif message == "GET_CURRENT_POS":
                try:
                    params = load_params_from_json()
                    print(f"Loaded params for GET_CURRENT_POS: {params['current_positions']}")
                    if params:
                        motor1_pos = params['current_positions']['motor1']
                        motor2_pos = params['current_positions']['motor2']
                        motor3_pos = params['current_positions']['motor3']
                        motor4_pos = params['current_positions']['motor4']
                        send_udp_message(f"SET_POS {motor1_pos},{motor2_pos},{motor3_pos},{motor4_pos}")
                        print(f"Sent SET_POS with positions: {motor1_pos},{motor2_pos},{motor3_pos},{motor4_pos}")
                        time.sleep(1.0)

                        preset_a_m1 = params['preset_a']['motor1']
                        preset_a_m2 = params['preset_a']['motor2']
                        preset_a_m3 = params['preset_a']['motor3']
                        preset_a_m4 = params['preset_a']['motor4']
                        send_udp_message(f"SET_PRESET_A {preset_a_m1},{preset_a_m2},{preset_a_m3},{preset_a_m4}")
                        print(f"Sent SET_PRESET_A with positions: {preset_a_m1},{preset_a_m2},{preset_a_m3},{preset_a_m4}")
                        time.sleep(1.0)

                        preset_b_m1 = params['preset_b']['motor1']
                        preset_b_m2 = params['preset_b']['motor2']
                        preset_b_m3 = params['preset_b']['motor3']
                        preset_b_m4 = params['preset_b']['motor4']
                        send_udp_message(f"SET_PRESET_B {preset_b_m1},{preset_b_m2},{preset_b_m3},{preset_b_m4}")
                        print(f"Sent SET_PRESET_B with positions: {preset_b_m1},{preset_b_m2},{preset_b_m3},{preset_b_m4}")
                        time.sleep(1.0)

                        send_udp_message(f"SET_MSSPEED {params['ms_speed']}")
                    else:
                        print("No JSON parameters available to send.")
                        send_udp_message(f"SET_POS 0,0,0,0")
                except FileNotFoundError:
                    print(f"Error: {PARAMS_FILE} not found, using default values.")
                    send_udp_message(f"SET_POS 0,0,0,0")
                except json.JSONDecodeError:
                    print(f"Error: {PARAMS_FILE} is corrupted, using default values.")
                    send_udp_message(f"SET_POS 0,0,0,0")
            if loop_state:
                if message == "PRESET_A_DONE":
                    time.sleep(0.5)
                    send_udp_message("RECALL_B")
                elif message == "PRESET_B_DONE":
                    time.sleep(0.5)
                    send_udp_message("RECALL_A")
        except Exception as e:
            print(f"UDP Receive Error: {e}")

# Function to update motor positions on Kivy labels
def update_motor_positions(message):
    try:
        positions = message.split("CURRENT_POS:")[1].strip().split(",")
        motor1_pos = int(positions[0])
        motor2_pos = int(positions[1])
        motor3_pos = int(positions[2])
        motor4_pos = int(positions[3])

        print(f"Parsed positions - Motor 1: {motor1_pos}, Motor 2: {motor2_pos}, Motor 3: {motor3_pos}, Motor 4: {motor4_pos}")

        current_pos1_label.text = f"Motor 1 Position: {motor1_pos}"
        current_pos2_label.text = f"Motor 2 Position: {motor2_pos}"
        current_pos3_label.text = f"Motor 3 Position: {motor3_pos}"
        current_pos4_label.text = f"Motor 4 Position: {motor4_pos}"
    except Exception as e:
        print(f"Error updating positions: {e}")

# Function to update preset positions
def update_preset_positions(message):
    global preset_a_slide, preset_b_slide
    parts = message.split("PRESET_POS_A:")[1].split("PRESET_POS_B:")
    preset_a_values = parts[0].strip().split(",")
    preset_b_values = parts[1].strip().split(",")

    preset_a_slide = int(preset_a_values[0])
    preset_b_slide = int(preset_b_values[0])

    preset_a_label.text = f"Preset A Position: {preset_a_slide}, {preset_a_values[1]}, {preset_a_values[2]}, {preset_a_values[3]}"
    preset_b_label.text = f"Preset B Position: {preset_b_slide}, {preset_b_values[1]}, {preset_b_values[2]}, {preset_b_values[3]}"

# Preset positions (updated dynamically)
preset_a_slide = 12641
preset_b_slide = 17384

# Flags to welcome if a preset is reached
preset_a_reached = False
preset_b_reached = False

# Define a tolerance to avoid small oscillations
tolerance = 600

# Helper function to apply deadzone
def apply_deadzone(value, deadzone_threshold):
    if abs(value) < deadzone_threshold:
        return 0.0
    return value

# Function to handle joystick input in a separate thread using 'inputs'
def joystick_input_thread():
    global x_axis_left, x_axis_right, y_axis_left, y_axis_right, trigger_left_mapped, trigger_right_mapped, last_button_states, virtualloop_enabled, invert_left, invert_right, invert_ry, gamepad, gamepad_initialized, sensitivity_right

    current_x_left_raw = 0.0
    current_y_left_raw = 0.0
    current_x_right_raw = 0.0
    current_y_right_raw = 0.0
    current_trigger_left_raw = 0.0
    current_trigger_right_raw = 0.0

    while True:
        if gamepad:
            try:
                events = get_gamepad()
                for event in events:
                    if event.ev_type == 'Absolute':
                        if event.code == 'ABS_X':
                            current_x_left_raw = event.state / 32767.0
                        elif event.code == 'ABS_Y':
                            current_y_left_raw = -event.state / 32767.0
                        elif event.code == 'ABS_RX':
                            current_x_right_raw = event.state / 32767.0
                        elif event.code == 'ABS_RY':
                            current_y_right_raw = -event.state / 32767.0
                        elif event.code == 'ABS_Z':
                            current_trigger_left_raw = event.state / 255.0
                        elif event.code == 'ABS_RZ':
                            current_trigger_right_raw = -(event.state / 255.0)

                        elif event.code == 'ABS_HAT0X':
                            if event.state == -1:
                                if not last_button_states.get('ABS_HAT0X_LEFT', False):
                                    send_udp_message("RECALL_A")
                                last_button_states['ABS_HAT0X_LEFT'] = True
                            elif event.state == 1:
                                if not last_button_states.get('ABS_HAT0X_RIGHT', False):
                                    send_udp_message("RECALL_B")
                                last_button_states['ABS_HAT0X_RIGHT'] = True
                            else:
                                last_button_states['ABS_HAT0X_LEFT'] = False
                                last_button_states['ABS_HAT0X_RIGHT'] = False
                        elif event.code == 'ABS_HAT0Y':
                            if event.state == -1:
                                if not last_button_states.get('ABS_HAT0Y_UP', False):
                                    virtualloop_enabled = True
                                    print("Virtualloop enabled")
                                last_button_states['ABS_HAT0Y_UP'] = True
                            elif event.state == 1:
                                if not last_button_states.get('ABS_HAT0Y_DOWN', False):
                                    virtualloop_enabled = False
                                    print("Virtualloop disabled")
                                last_button_states['ABS_HAT0Y_DOWN'] = True
                            else:
                                last_button_states['ABS_HAT0Y_UP'] = False
                                last_button_states['ABS_HAT0Y_DOWN'] = False

                    elif event.ev_type == 'Key':
                        button_name = event.code
                        is_pressed = (event.state == 1)
                        current_state = is_pressed
                        if current_state and not last_button_states.get(button_name, False):
                            if button_name == 'BTN_EAST': send_udp_message("SAVE_B")
                            elif button_name == 'BTN_WEST': send_udp_message("SAVE_A")
                            elif button_name == 'BTN_NORTH': send_udp_message("UPDATE_LOOP true"); set_loop_state(True)
                            elif button_name == 'BTN_SOUTH': send_udp_message("UPDATE_LOOP false"); set_loop_state(False)
                            elif button_name == 'BTN_TR': send_udp_message("INCREASE_SPEED")
                            elif button_name == 'BTN_TL': send_udp_message("DECREASE_SPEED")
                        last_button_states[button_name] = current_state

                # Apply deadzone and sensitivity
                x_axis_left = apply_deadzone(current_x_left_raw, DEADZONE)
                y_axis_left = apply_deadzone(current_y_left_raw, DEADZONE)
                x_axis_right = apply_deadzone(current_x_right_raw, DEADZONE) * sensitivity_right
                y_axis_right = apply_deadzone(current_y_right_raw, DEADZONE) * sensitivity_right
                trigger_left_mapped = apply_deadzone(current_trigger_left_raw, DEADZONE)
                trigger_right_mapped = apply_deadzone(current_trigger_right_raw, DEADZONE)

                if invert_left:
                    x_axis_left = -x_axis_left
                    y_axis_left = -y_axis_left
                if invert_right:
                    x_axis_right = -x_axis_right
                if invert_ry:
                    y_axis_right = -y_axis_right

                if virtualloop_enabled:
                    x_axis_left = 0
                    y_axis_left = 0

            except UnpluggedError:
                print("Gamepad unplugged. Re-initializing...")
                gamepad = None
                gamepad_initialized = False
                gamepad_status_update("Not Detected")
                time.sleep(2)
                continue
            except Exception as e:
                print(f"Error processing gamepad input: {e}")
        else:
            if not gamepad_initialized:
                print("Attempting to re-initialize gamepad...")
                if initialize_gamepad():
                    gamepad_status_update("Connected")
                else:
                    gamepad_status_update("Not Detected")
            time.sleep(1)

# This function is called by Kivy to send UDP messages based on thread-updated values
def send_joystick_data(dt):
    global x_axis_left, x_axis_right, y_axis_right, trigger_left_mapped, trigger_right_mapped
    if gamepad:
        message = f"SET_JOYSTICK {x_axis_left:.3f},{x_axis_right:.3f},{y_axis_right:.3f},{trigger_left_mapped:.3f},{trigger_right_mapped:.3f}"
        print(f"Sending: {message}")
        send_udp_message(message)
        send_udp_message("SEND_CURRENT_POS")
    else:
        joystick_input_label.text = "Joystick Input: Not Connected"

# This function is called repeatedly to control the motor behavior
def virtualloop(dt):
    global virtualloop_enabled, preset_a_slide, preset_b_slide, preset_a_reached, preset_b_reached, trigger_left_mapped, trigger_right_mapped

    if virtualloop_enabled:
        try:
            motor1_pos = int(current_pos1_label.text.split(":")[1].strip())
            if not preset_a_reached:
                target_position = preset_a_slide
            else:
                target_position = preset_b_slide

            if motor1_pos < target_position:
                direction = 1.000
            else:
                direction = -1.000

            print(f"Virualloop: motor1_pos={motor1_pos}, target={target_position}, direction={direction}, a_reached={preset_a_reached}, b_reached={preset_b_reached}")
            send_udp_message(f"SET_JOYSTICK {direction},{x_axis_right:.3f},{y_axis_right:.3f},{trigger_left_mapped:.3f},{trigger_right_mapped:.3f}")

            if motor1_pos >= target_position - tolerance and motor1_pos <= target_position + tolerance:
                if target_position == preset_a_slide and not preset_a_reached:
                    preset_a_reached = True
                    print("Preset A reached. Moving to Preset B.")
                    preset_b_reached = False
                elif target_position == preset_b_slide and not preset_b_reached:
                    preset_b_reached = True
                    print("Preset B reached. Moving to Preset A.")
                    preset_a_reached = False
        except Exception as e:
            print(f"Virualloop Error: {e}")

# Kivy UI class
class ControlApp(App):
    def build(self):
        global ms_speed, current_pos1_label, current_pos2_label, current_pos3_label, current_pos4_label, preset_a_label, preset_b_label, ms_speed_label, UDP_IP, joystick_status_label, error_label, joystick_input_label, sensitivity_label, sensitivity_right

        set_params_file_path()

        layout = BoxLayout(orientation='vertical')

        joystick_status_label = Label(text="Joystick Status: Initializing...")
        joystick_input_label = Label(text="Joystick Input: Not Connected")
        error_label = Label(text="JSON Status: Not Loaded")
        current_pos1_label = Label(text="Motor 1 Position: 0")
        current_pos2_label = Label(text="Motor 2 Position: 0")
        current_pos3_label = Label(text="Motor 3 Position: 0")
        current_pos4_label = Label(text="Motor 4 Position: 0")
        preset_a_label = Label(text="Preset A Position: 0, 0, 0, 0")
        preset_b_label = Label(text="Preset B Position: 0, 0, 0, 0")
        ms_speed_label = Label(text=f"msSpeed: {ms_speed}")

        layout.add_widget(joystick_status_label)
        layout.add_widget(joystick_input_label)
        layout.add_widget(error_label)
        layout.add_widget(current_pos1_label)
        layout.add_widget(current_pos2_label)
        layout.add_widget(current_pos3_label)
        layout.add_widget(current_pos4_label)
        layout.add_widget(preset_a_label)
        layout.add_widget(preset_b_label)
        layout.add_widget(ms_speed_label)

        # Create UDP IP input field and save button before loading params
        udp_layout = BoxLayout(orientation='horizontal', size_hint=(1, None), height=40)
        udp_ip_input = TextInput(text=UDP_IP, multiline=False, size_hint=(0.7, 1))
        udp_ip_save_button = Button(text="Save UDP IP", size_hint=(0.3, 1))
        def save_udp_ip(instance):
            global UDP_IP
            new_ip = udp_ip_input.text.strip()
            if new_ip:
                UDP_IP = new_ip
                save_params_to_json()
                print(f"UDP IP updated to: {UDP_IP}")
        udp_ip_save_button.bind(on_press=save_udp_ip)
        udp_layout.add_widget(udp_ip_input)
        udp_layout.add_widget(udp_ip_save_button)

        # Load parameters from JSON after creating udp_ip_input
        params = load_params_from_json()
        if params:
            print("Loaded parameters from JSON:", params)
            current_pos1_label.text = f"Motor 1 Position: {params['current_positions']['motor1']}"
            current_pos2_label.text = f"Motor 2 Position: {params['current_positions']['motor2']}"
            current_pos3_label.text = f"Motor 3 Position: {params['current_positions']['motor3']}"
            current_pos4_label.text = f"Motor 4 Position: {params['current_positions']['motor4']}"
            preset_a_label.text = f"Preset A Position: {params['preset_a']['motor1']}, {params['preset_a']['motor2']}, {params['preset_a']['motor3']}, {params['preset_a']['motor4']}"
            preset_b_label.text = f"Preset B Position: {params['preset_b']['motor1']}, {params['preset_b']['motor2']}, {params['preset_b']['motor3']}, {params['preset_b']['motor4']}"
            ms_speed_label.text = f"msSpeed: {params['ms_speed']}"
            ms_speed = params['ms_speed']
            if "udp_ip" in params:
                UDP_IP = params["udp_ip"]
                udp_ip_input.text = UDP_IP  # Now safe to access
            invert_left = params.get("invert_left", False)
            invert_right = params.get("invert_right", False)
            invert_ry = params.get("invert_ry", False)
            sensitivity_right = params.get("sensitivity_right", 1.0)
        else:
            print("No parameters loaded from JSON; using defaults.")

        # Now create the sensitivity label after sensitivity_right is loaded
        sensitivity_label = Label(text=f"Right Joystick Sensitivity: {sensitivity_right:.1f}")
        layout.add_widget(sensitivity_label)

        buttons = [
            ("Save Preset A", lambda x: send_udp_message("SAVE_A")),
            ("Save Preset B", lambda x: send_udp_message("SAVE_B")),
            ("Recall Preset A", lambda x: send_udp_message("RECALL_A")),
            ("Recall Preset B", lambda x: send_udp_message("RECALL_B")),
            ("Loop Presets ON", lambda x: send_udp_message("UPDATE_LOOP true") or set_loop_state(True)),
            ("Loop Presets OFF", lambda x: send_udp_message("UPDATE_LOOP false") or set_loop_state(False)),
            ("Virtualloop ON", lambda x: set_virtualloop_state(True)),
            ("Virtualloop OFF", lambda x: set_virtualloop_state(False)),
            ("Invert Left Joystick", lambda x: toggle_invert_left()),
            ("Invert Right Joystick", lambda x: toggle_invert_right()),
            ("Decrease Sensitivity", lambda x: adjust_sensitivity(-0.1)),
            ("Increase Sensitivity", lambda x: adjust_sensitivity(0.1)),
        ]
        for text, cmd in buttons:
            layout.add_widget(Button(text=text, on_press=cmd))

        # Add the UDP layout to the main layout
        layout.add_widget(udp_layout)

        from kivy.core.window import Window
        Window.bind(on_minimize=self.on_minimize)
        Window.bind(on_restore=self.on_restore)
        Window.bind(on_focus=self.on_focus)

        global joystick_thread
        joystick_thread = threading.Thread(target=joystick_input_thread, daemon=True)
        joystick_thread.start()

        if gamepad_initialized:
            joystick_status_label.text = "Joystick Status: Connected"
        else:
            joystick_status_label.text = "Joystick Status: Not Detected"

        return layout

    def on_minimize(self, *args):
        print("Window minimized")
        return False

    def on_restore(self, *args):
        print("Window restored")

    def on_focus(self, window, focus):
        print(f"Window focus changed: {focus}")
        return False

    def on_pause(self):
        return True

    def on_stop(self):
        global joystick_thread
        sock.close()
        if joystick_thread:
            joystick_thread.join(timeout=1.0)

# Function to adjust right joystick sensitivity
def adjust_sensitivity(change):
    global sensitivity_right, sensitivity_label
    sensitivity_right = max(0.1, min(1.0, sensitivity_right + change))  # Bound between 0.1 and 1.0
    if sensitivity_label:
        sensitivity_label.text = f"Right Joystick Sensitivity: {sensitivity_right:.1f}"
    save_params_to_json()
    print(f"Adjusted sensitivity to: {sensitivity_right}")

# Function to start/stop looping between presets
def set_loop_state(state):
    global loop_state
    loop_state = state
    if state:
        send_udp_message("RECALL_A")

# Function to start/stop virtualloop
def set_virtualloop_state(state):
    global virtualloop_enabled
    virtualloop_enabled = state
    print(f"Virtualloop set to {state}")

# Functions to toggle inversion
def toggle_invert_left():
    global invert_left
    invert_left = not invert_left
    print(f"Left Joystick Inversion: {invert_left}")

def toggle_invert_right():
    global invert_right
    invert_right = not invert_right
    print(f"Right Joystick Inversion: {invert_right}")

# Function to update gamepad status in the Kivy UI
def gamepad_status_update(status):
    global joystick_status_label
    joystick_status_label.text = f"Joystick Status: {status}"

# Configure Kivy to not pause on minimize or focus loss
Config.set('kivy', 'pause_on_minimize', 0)

# Schedule Kivy updates
Clock.schedule_interval(send_joystick_data, 0.1)
Clock.schedule_interval(virtualloop, 0.1)
Clock.schedule_interval(save_params_periodically, 1.0)

if __name__ == '__main__':
    udp_receive_thread = threading.Thread(target=receive_udp_messages, daemon=True)
    udp_receive_thread.start()
    ControlApp().run()