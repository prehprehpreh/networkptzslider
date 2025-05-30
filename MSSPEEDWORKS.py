import socket
import pygame
import threading
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.clock import Clock
import time
import json
import os

# UDP Configuration
UDP_IP = "192.168.5.60"
UDP_PORT_SEND = 44158
UDP_PORT_RECEIVE = 44159

# Declare global variables
global x_axis_right, y_axis_right, loop_state, virtualloop_enabled, preset_a_slide, preset_b_slide, trigger_left_mapped, trigger_right_mapped, ms_speed, invert_left, invert_right
global initial_setup_complete
x_axis_right = 0.0
y_axis_right = 0.0
trigger_left_mapped = 0.0
trigger_right_mapped = 0.0
ms_speed = 800
invert_left = False
invert_right = False
initial_setup_complete = False

# Initialize UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT_RECEIVE))

# Initialize Pygame for joystick input
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected! Exiting...")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Deadzone threshold
DEADZONE = 0.01

# Control states
loop_state = False
virtualloop_enabled = False

# Button state tracking to avoid spamming
last_button_states = [False] * joystick.get_numbuttons()

# File path for saving motor parameters
PARAMS_FILE = r"C:\Users\prehp\OneDrive\Documents\Controller2\motor_params.json"

# Function to save motor parameters to JSON file
def save_params_to_json():
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
            "udp_ip": UDP_IP
        }
        with open(PARAMS_FILE, "w") as f:
            json.dump(params, f, indent=4)
        print(f"Saved parameters to {os.path.abspath(PARAMS_FILE)}")
    except Exception as e:
        print(f"Error saving JSON file: {e}")

# Function to load motor parameters from JSON file
def load_params_from_json():
    if os.path.exists(PARAMS_FILE):
        print(f"Found JSON file at {os.path.abspath(PARAMS_FILE)}")
        try:
            with open(PARAMS_FILE, "r") as f:
                params = json.load(f)
            return params
        except Exception as e:
            print(f"Error loading JSON file: {e}")
            return None
    print(f"No JSON file found at {os.path.abspath(PARAMS_FILE)}")
    return None

# Function to send UDP message
def send_udp_message(message):
    print(f"Sending: {message}")
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
                    with open(PARAMS_FILE, "r") as f:
                        params = json.load(f)
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
        
        current_pos1_label.text = f"Motor 1 Position: {motor1_pos}"
        current_pos2_label.text = f"Motor 2 Position: {motor2_pos}"
        current_pos3_label.text = f"Motor 3 Position: {motor3_pos}"
        current_pos4_label.text = f"Motor 4 Position: {motor4_pos}"
        save_params_to_json()
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
    save_params_to_json()

# Preset positions (updated dynamically)
preset_a_slide = 12641
preset_b_slide = 17384

# Flags to welcome if a preset is reached
preset_a_reached = False
preset_b_reached = False

# Define a tolerance to avoid small oscillations
tolerance = 600

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
            print(f"Virtualloop Error: {e}")

# Function to send joystick data with trigger values, button logging, and controls
def send_joystick_data(dt):
    global x_axis_right, y_axis_right, trigger_left_mapped, trigger_right_mapped, last_button_states, virtualloop_enabled, invert_left, invert_right
    pygame.event.pump()
    
    # Get joystick axes
    x_axis_left = joystick.get_axis(0)
    y_axis_left = joystick.get_axis(1)
    x_axis_right = joystick.get_axis(2)
    y_axis_right = joystick.get_axis(3)
    trigger_left = joystick.get_axis(4)
    trigger_right = joystick.get_axis(5)

    # Apply deadzone and map triggers
    x_axis_left = 0 if abs(x_axis_left) < DEADZONE else x_axis_left
    y_axis_left = 0 if abs(y_axis_left) < DEADZONE else y_axis_left
    x_axis_right = 0 if abs(x_axis_right) < DEADZONE else x_axis_right
    y_axis_right = 0 if abs(y_axis_right) < DEADZONE else y_axis_right
    trigger_left_mapped = 0 if trigger_left < -0.9 else (trigger_left - (-1)) / 2 * -1
    trigger_right_mapped = 0 if trigger_right < -0.9 else (trigger_right - (-1)) / 2

    # Apply inversion if toggled
    if invert_left:
        x_axis_left = -x_axis_left
        y_axis_left = -y_axis_left
    if invert_right:
        x_axis_right = -x_axis_right
        y_axis_right = -y_axis_right

    # Log all button presses
    for i in range(joystick.get_numbuttons()):
        if joystick.get_button(i):
            print(f"Button {i} pressed")

    # Button controls with press detection
    button_states = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

    # Circle (Button 1): Save Preset B
    if button_states[1] and not last_button_states[1]:
        send_udp_message("SAVE_B")

    # Square (Button 2): Save Preset A
    if button_states[2] and not last_button_states[2]:
        send_udp_message("SAVE_A")

    # D-pad Left (Button 13): Recall Preset A
    if button_states[13] and not last_button_states[13]:
        send_udp_message("RECALL_A")

    # D-pad Right (Button 14): Recall Preset B
    if button_states[14] and not last_button_states[14]:
        send_udp_message("RECALL_B")

    # Triangle (Button 3): Loop Presets ON (same as Kivy button)
    if button_states[3] and not last_button_states[3]:
        send_udp_message("UPDATE_LOOP true")
        set_loop_state(True)

    # X (Button 0): Loop Presets OFF (same as Kivy button)
    if button_states[0] and not last_button_states[0]:
        send_udp_message("UPDATE_LOOP false")
        set_loop_state(False)

    # D-pad Up (Button 11): Set virtualloop true
    if button_states[11] and not last_button_states[11]:
        virtualloop_enabled = True
        print("Virtualloop enabled")

    # D-pad Down (Button 12): Set virtualloop false
    if button_states[12] and not last_button_states[12]:
        virtualloop_enabled = False
        print("Virtualloop disabled")

    # Button 10: Increase msSpeed
    if button_states[10] and not last_button_states[10]:
        send_udp_message("INCREASE_SPEED")

    # Button 9: Decrease msSpeed
    if button_states[9] and not last_button_states[9]:
        send_udp_message("DECREASE_SPEED")

    # Update last button states
    last_button_states = button_states

    # Disable left joystick movement when virtualloop is enabled
    if virtualloop_enabled:
        x_axis_left = 0

    # Send UDP message with 5 values: x_left, x_right, y_right, trigger_left, trigger_right
    message = f"SET_JOYSTICK {x_axis_left:.3f},{x_axis_right:.3f},{y_axis_right:.3f},{trigger_left_mapped:.3f},{trigger_right_mapped:.3f}"
    send_udp_message(message)

    # Automatically request current position
    send_udp_message("SEND_CURRENT_POS")

# Kivy UI class
class ControlApp(App):
    def build(self):
        global ms_speed, current_pos1_label, current_pos2_label, current_pos3_label, current_pos4_label, preset_a_label, preset_b_label, ms_speed_label, UDP_IP
        
        layout = BoxLayout(orientation='vertical')
        
        current_pos1_label = Label(text="Motor 1 Position: 0")
        current_pos2_label = Label(text="Motor 2 Position: 0")
        current_pos3_label = Label(text="Motor 3 Position: 0")
        current_pos4_label = Label(text="Motor 4 Position: 0")
        preset_a_label = Label(text="Preset A Position: 0, 0, 0, 0")
        preset_b_label = Label(text="Preset B Position: 0, 0, 0, 0")
        ms_speed_label = Label(text=f"msSpeed: {ms_speed}")
        
        layout.add_widget(current_pos1_label)
        layout.add_widget(current_pos2_label)
        layout.add_widget(current_pos3_label)
        layout.add_widget(current_pos4_label)
        layout.add_widget(preset_a_label)
        layout.add_widget(preset_b_label)
        layout.add_widget(ms_speed_label)
        
        buttons = [
            ("Save Preset A", "SAVE_A"),
            ("Save Preset B", "SAVE_B"),
            ("Recall Preset A", "RECALL_A"),
            ("Recall Preset B", "RECALL_B"),
            ("Loop Presets ON", lambda x: send_udp_message("UPDATE_LOOP true") or set_loop_state(True)),
            ("Loop Presets OFF", lambda x: send_udp_message("UPDATE_LOOP false") or set_loop_state(False)),
            ("Virtualloop ON", lambda x: set_virtualloop_state(True)),
            ("Virtualloop OFF", lambda x: set_virtualloop_state(False)),
            ("Invert Left Joystick", lambda x: toggle_invert_left()),
            ("Invert Right Joystick", lambda x: toggle_invert_right()),
        ]
        for text, cmd in buttons:
            layout.add_widget(Button(text=text, on_press=lambda x, c=cmd: c(x)))
        
        # UDP IP input field and save button
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
        layout.add_widget(udp_layout)
        
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
                udp_ip_input.text = UDP_IP
        else:
            print("No parameters loaded from JSON; using defaults.")
        
        return layout

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

# New functions to toggle inversion
def toggle_invert_left():
    global invert_left
    invert_left = not invert_left
    print(f"Left Joystick Inversion: {invert_left}")

def toggle_invert_right():
    global invert_right
    invert_right = not invert_right
    print(f"Right Joystick Inversion: {invert_right}")

Clock.schedule_interval(send_joystick_data, 0.1)
Clock.schedule_interval(virtualloop, 0.1)

if __name__ == '__main__':
    udp_receive_thread = threading.Thread(target=receive_udp_messages, daemon=True)
    udp_receive_thread.start()
    ControlApp().run()