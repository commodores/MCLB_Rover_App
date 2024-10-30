import sys
from flask import flash
import pygame
from .rover_connect import rover_connection

pygame.init()
pygame.joystick.init()

#Create empty list for joystics
joystick = []


joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Joysticks Connected")



# Function to send manual control to Pixhawk
def send_manual_control(roll, pitch, throttle, yaw, buttons):
    rover_connection.mav.manual_control_send(
        rover_connection.target_system,    # Target system (Pixhawk)
        int(roll * 1000),        # X-axis (roll)
        int(pitch * 1000),       # Y-axis (pitch)
        int(throttle * 1000),    # Z-axis (throttle)
        int(yaw * 1000),         # R-axis (yaw)
        buttons                  # Buttons bitmask
    )

# Map joystick inputs to MAVLink control signals
run = True
while True:


 # Check the number of joysticks and info
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick detected.")

    print(f"Controllers:"(joystick_count))
    print(f"Battery Level:"(joystick.get_power_level))
    print(f"Controller Type:"(joystick.get_name))
    print(f"Number of axes:"(joystick.get_numaxes))


    # Left joystick: Throttle and Yaw
    throttle = joystick.get_axis(1)  # Y-axis (forward/backward) - Throttle
    yaw = joystick.get_axis(0)       # X-axis (left/right) - Yaw

    
    # Right joystick: Roll and Pitch
    roll = joystick.get_axis(3)      # X-axis (left/right) - Roll
    pitch = joystick.get_axis(4)     # Y-axis (forward/backward) - Pitch
    
    # Read button presses (optional)
    button_bitmask = 0
    if joystick.get_button(0):  # Button 0 for arming/disarming
        button_bitmask |= (1 << 0)

    # Send manual control to Pixhawk
    send_manual_control(roll, pitch, throttle, yaw, button_bitmask)

    def handle_button_press():
        # Button 0: Arm the drone
        if joystick.get_button(0):
            print("Arming the Rover")
            flash("Arming the Rover")
            rover_connection.arducopter_arm()

        # Button 1: Disarm the drone
        if joystick.get_button(1):
            print("Disarming the Rover")
            flash("Diarming the Rover")
            rover_connection.arducopter_disarm()

        # Button 2: Switch flight mode
        if joystick.get_button(2):
            print("Switching to GUIDED mode")
            flash("Switching to GUIDED mode")
            rover_connection.set_mode_px4('GUIDED')

    # In the main loop:
    handle_button_press()


    #event handler
    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            print(event)
            joy = pygame.joystick.Joystick(event.device_index)
            joystick.append(joy)
        if event.type == pygame.QUIT:
            run = False

    
   
    
    pygame.quit()