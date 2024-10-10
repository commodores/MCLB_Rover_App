import pygame
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

# Initialize pygame and the joystick
def joystick2():
    pygame.init()
    pygame.joystick.init()

    joystick = pygame.joystick.Joystick(1)
    joystick.init()

    the_connection.wait_heartbeat()

    # Function to send manual control to Pixhawk
    def send_manual_control(roll, pitch, throttle, yaw, buttons):
        the_connection.mav.manual_control_send(
            the_connection.target_system,    # Target system (Pixhawk)
            int(roll * 1000),        # X-axis (roll)
            int(pitch * 1000),       # Y-axis (pitch)
            int(throttle * 1000),    # Z-axis (throttle)
            int(yaw * 1000),         # R-axis (yaw)
            buttons                  # Buttons bitmask
        )

    # Map joystick inputs to MAVLink control signals
    while True:
        pygame.event.pump()
        
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
                print("Arming the drone")
                the_connection.arducopter_arm()

            # Button 1: Disarm the drone
            if joystick.get_button(1):
                print("Disarming the drone")
                the_connection.arducopter_disarm()

            # Button 2: Switch flight mode
            if joystick.get_button(2):
                print("Switching to GUIDED mode")
                the_connection.set_mode_px4('GUIDED')

        # In the main loop:
        handle_button_press()
