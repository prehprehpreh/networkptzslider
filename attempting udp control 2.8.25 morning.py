import socket
import pygame
import threading
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock

# UDP Configuration
UDP_IP = "192.168.0.60"  # Arduino's IP Address
UDP_PORT_SEND = 44158  # Port for sending
UDP_PORT_RECEIVE = 44159  # Port for receiving (must match Arduino's sending port)

# Initialize UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT_RECEIVE))  # Bind to all interfaces to receive data

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

def send_udp_message(message):
    print(f"Sending: {message}")
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT_SEND))

# Function to receive UDP messages and print them
import time

# Function to handle received UDP messages
def receive_udp_messages():
    global loop_state
    while True:
        try:
            # Receive the message (1024 bytes buffer size)
            data, addr = sock.recvfrom(1024)
            message = data.decode()  # Decode message to string

            # Print the received UDP message
            print(f"Received from {addr}: {message}")

            # Check if we received a confirmation that preset A or B was reached
            if loop_state:
                if message == "PRESET_A_DONE":
                    print("Delay before sending RECALL_B...")
                    time.sleep(0.5)  # Delay for 2 seconds (adjust as needed)
                    send_udp_message("RECALL_B")  # Move to Preset B
                elif message == "PRESET_B_DONE":
                    print("Delay before sending RECALL_A...")
                    time.sleep(0.5)  # Delay for 2 seconds (adjust as needed)
                    send_udp_message("RECALL_A")  # Move to Preset A

        except Exception as e:
            print(f"UDP Receive Error: {e}")

# Start the UDP receive thread
udp_receive_thread = threading.Thread(target=receive_udp_messages, daemon=True)
udp_receive_thread.start()

def send_joystick_data(dt):
    pygame.event.pump()

    # Check joystick axes
    x_axis_left = joystick.get_axis(0)
    x_axis_left = 0 if abs(x_axis_left) < DEADZONE else x_axis_left
    x_axis_right = joystick.get_axis(2)
    x_axis_right = 0 if abs(x_axis_right) < DEADZONE else x_axis_right
    y_axis_right = joystick.get_axis(3)
    y_axis_right = 0 if abs(y_axis_right) < DEADZONE else y_axis_right
    message = f"SET_JOYSTICK {x_axis_left:.3f},{x_axis_right:.3f},{y_axis_right:.3f}"
    send_udp_message(message)

    # Check for button presses (Print button states for debugging)
    for i in range(joystick.get_numbuttons()):
        button_state = joystick.get_button(i)
        if button_state:
            print(f"Button {i} is pressed.")

    #increase speed on button press (D-pad)
    if joystick.get_button(11):  
        print("Increasing speed.")
        send_udp_message("INCREASE_SPEED")
    elif joystick.get_button(12):  
        print("Decreasing speed.")
        send_udp_message("DECREASE_SPEED")
    elif joystick.get_button(2):  
        print("Save_Preset_A")
        send_udp_message("SAVE_A")
    elif joystick.get_button(1):  
        print("Save_Preset_B.")
        send_udp_message("SAVE_B")
    elif joystick.get_button(13):  
        print("Recall Preset A")
        send_udp_message("RECALL_A")
    elif joystick.get_button(14):  
        print("Recall Preset B")
        send_udp_message("RECALL_B")
    elif joystick.get_button(3):  
        print("Loop_state_True")
        set_loop_state(True)
    elif joystick.get_button(0):  
        print("Loop_state_False")
        set_loop_state(False)

class ControlApp(App):
    def build(self):
        layout = BoxLayout(orientation='vertical')
        buttons = [
            ("Save Position", "SAVE_POS"),
            ("Recall Position", "RECALL_POS"),
            ("Save Preset A", "SAVE_A"),
            ("Save Preset B", "SAVE_B"),
            ("Recall Preset A", "RECALL_A"),
            ("Recall Preset B", "RECALL_B"),
            ("Loop Presets ON", lambda x: set_loop_state(True)),
            ("Loop Presets OFF", lambda x: set_loop_state(False)),
        ]
        for text, cmd in buttons:
            layout.add_widget(Button(text=text, on_press=lambda x, c=cmd: send_udp_message(c) if isinstance(c, str) else c(x)))
        return layout

# Function to start/stop looping between presets
def set_loop_state(state):
    global loop_state
    loop_state = state
    if state:
        print("Looping between A and B started.")
        send_udp_message("RECALL_A")  # Start by going to Preset A
    else:
        print("Looping between A and B stopped.")
Clock.schedule_interval(send_joystick_data, 0.1)

if __name__ == '__main__':
    loop_state = False
    ControlApp().run()
