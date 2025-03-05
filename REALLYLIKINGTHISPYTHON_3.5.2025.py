import socket
import pygame
import threading
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.clock import Clock
import time

# UDP Configuration
UDP_IP = "192.168.1.69"  # Arduino's IP Address
UDP_PORT_SEND = 44158  # Port for sending
UDP_PORT_RECEIVE = 44159  # Port for receiving

# Declare global variables
global x_axis_right, y_axis_right, loop_state, virtualloop_enabled, preset_a_slide, preset_b_slide, trigger_left_mapped, trigger_right_mapped, ms_speed
x_axis_right = 0.0
y_axis_right = 0.0
trigger_left_mapped = 0.0
trigger_right_mapped = 0.0
ms_speed = 800  # Initial value matches Arduino's default

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

# D-pad state tracking to avoid spamming
last_dpad_up_state = False
last_dpad_down_state = False

def send_udp_message(message):
    print(f"Sending: {message}")
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT_SEND))

# Function to handle received UDP messages
def receive_udp_messages():
    global loop_state, ms_speed
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode()
            print(f"Received from {addr}: {message}")

            if "CURRENT_POS:" in message:
                update_motor_positions(message)
            elif "PRESET_POS_A:" in message and "PRESET_POS_B:" in message:
                update_preset_positions(message)
            elif "MSSPEED:" in message:
                ms_speed = float(message.split("MSSPEED:")[1])
                ms_speed_label.text = f"msSpeed: {ms_speed}"  # Update Kivy label

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
        
        current_pos1_label.text = f"Motor 1 Position: {motor1_pos}"
        current_pos2_label.text = f"Motor 2 Position: {motor2_pos}"
        current_pos3_label.text = f"Motor 3 Position: {motor3_pos}"
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
    
    preset_a_label.text = f"Preset A Position: {preset_a_slide}, {preset_a_values[1]}, {preset_a_values[2]}"
    preset_b_label.text = f"Preset B Position: {preset_b_slide}, {preset_b_values[1]}, {preset_b_values[2]}"

# Preset positions (updated dynamically)
preset_a_slide = 12641
preset_b_slide = 17384

# Flags to track if a preset is reached
preset_a_reached = False
preset_b_reached = False

# Define a tolerance to avoid small oscillations
tolerance = 600

# This function is called repeatedly to control the motor behavior
def virtualloop(dt):
    global virtualloop_enabled, preset_a_slide, preset_b_slide, preset_a_reached, preset_b_reached, trigger_left_mapped, trigger_right_mapped

    if virtualloop_enabled:
        try:
            motor1_pos = int(current_pos1_label.text.split(":")[1].strip())  # Get current motor position

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

# Function to send joystick data with trigger values, button logging, and speed control
def send_joystick_data(dt):
    global x_axis_right, y_axis_right, trigger_left_mapped, trigger_right_mapped, last_dpad_up_state, last_dpad_down_state
    pygame.event.pump()
    
    # Get joystick axes
    x_axis_left = joystick.get_axis(0)
    y_axis_left = joystick.get_axis(1)
    x_axis_right = joystick.get_axis(2)
    y_axis_right = joystick.get_axis(3)
    trigger_left = joystick.get_axis(4)  # Left Trigger (Axis 4)
    trigger_right = joystick.get_axis(5)  # Right Trigger (Axis 5)

    # Apply deadzone and map triggers
    x_axis_left = 0 if abs(x_axis_left) < DEADZONE else x_axis_left
    y_axis_left = 0 if abs(y_axis_left) < DEADZONE else y_axis_left
    x_axis_right = 0 if abs(x_axis_right) < DEADZONE else x_axis_right
    y_axis_right = 0 if abs(y_axis_right) < DEADZONE else y_axis_right
    trigger_left_mapped = 0 if trigger_left < -0.9 else (trigger_left - (-1)) / 2 * -1  # -1 to 1 -> 0 to -1
    trigger_right_mapped = 0 if trigger_right < -0.9 else (trigger_right - (-1)) / 2  # -1 to 1 -> 0 to 1

    # Log all button presses
    for i in range(joystick.get_numbuttons()):
        if joystick.get_button(i):
            print(f"Button {i} pressed")

    # D-pad control for speed (PS4 D-pad: up = button 11, down = button 12)
    dpad_up = joystick.get_button(11)  # Up button
    dpad_down = joystick.get_button(12)  # Down button

    # Detect button press (not hold) to avoid spamming
    if dpad_up and not last_dpad_up_state:
        send_udp_message("INCREASE_SPEED")
    if dpad_down and not last_dpad_down_state:
        send_udp_message("DECREASE_SPEED")

    # Update last states
    last_dpad_up_state = dpad_up
    last_dpad_down_state = dpad_down

    # Disable left joystick movement when virtualloop is enabled
    if virtualloop_enabled:
        x_axis_left = 0  # Disable left joystick (only affecting motor 1)

    # Send UDP message with 5 values: x_left, x_right, y_right, trigger_left, trigger_right
    message = f"SET_JOYSTICK {x_axis_left:.3f},{x_axis_right:.3f},{y_axis_right:.3f},{trigger_left_mapped:.3f},{trigger_right_mapped:.3f}"
    send_udp_message(message)
    send_udp_message("SEND_CURRENT_POS")

# Kivy UI class
class ControlApp(App):
    def build(self):
        layout = BoxLayout(orientation='vertical')
        global current_pos1_label, current_pos2_label, current_pos3_label, preset_a_label, preset_b_label, ms_speed_label
        
        current_pos1_label = Label(text="Motor 1 Position: 0")
        current_pos2_label = Label(text="Motor 2 Position: 0")
        current_pos3_label = Label(text="Motor 3 Position: 0")
        preset_a_label = Label(text="Preset A Position: 0, 0, 0")
        preset_b_label = Label(text="Preset B Position: 0, 0, 0")
        ms_speed_label = Label(text=f"msSpeed: {ms_speed}")
        
        layout.add_widget(current_pos1_label)
        layout.add_widget(current_pos2_label)
        layout.add_widget(current_pos3_label)
        layout.add_widget(preset_a_label)
        layout.add_widget(preset_b_label)
        layout.add_widget(ms_speed_label)
        
        buttons = [
            ("Save Preset A", "SAVE_A"),
            ("Save Preset B", "SAVE_B"),
            ("Recall Preset A", "RECALL_A"),
            ("Recall Preset B", "RECALL_B"),
            ("Loop Presets ON", lambda x: set_loop_state(True)),
            ("Loop Presets OFF", lambda x: set_loop_state(False)),
            ("Virtualloop ON", lambda x: set_virtualloop_state(True)),
            ("Virtualloop OFF", lambda x: set_virtualloop_state(False)),
        ]
        for text, cmd in buttons:
            layout.add_widget(Button(text=text, on_press=lambda x, c=cmd: send_udp_message(c) if isinstance(c, str) else c(x)))
        
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

Clock.schedule_interval(send_joystick_data, 0.1)
Clock.schedule_interval(virtualloop, 0.1)

if __name__ == '__main__':
    udp_receive_thread = threading.Thread(target=receive_udp_messages, daemon=True)
    udp_receive_thread.start()
    ControlApp().run()