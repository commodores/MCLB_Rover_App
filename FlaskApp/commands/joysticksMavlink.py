from pymavlink import mavutil
from commands import joystick
import pygame
from commands import joysticksTwo

# Connect to Pixhawk
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for a heartbeat to establish communication
master.wait_heartbeat()

# Function to send manual control command
def send_manual_control(x, y, z, r, buttons):
    master.mav.manual_control_send(
        master.target_system,  # Target system ID
        int(x * 1000),         # X-axis (throttle/forward-backward)
        int(y * 1000),         # Y-axis (left-right/strafe)
        int(z * 1000),         # Z-axis (up-down)
        int(r * 1000),         # R-axis (yaw)
        buttons                # Button bitmask (for arming, mode switch, etc.)
    )

# Map joystick inputs to control signals and send them to Pixhawk
while True:
    pygame.event.pump()
    x = joystick.get_axis(0)  # Left stick X-axis (e.g., throttle)
    y = joystick.get_axis(1)  # Left stick Y-axis (e.g., yaw)
    z = joystick.get_axis(3)  # Right stick Y-axis (e.g., pitch)
    r = joystick.get_axis(2)  # Right stick X-axis (e.g., roll)
    
    # Send these values to Pixhawk
    send_manual_control(x, y, z, r, buttons=0)  # You can also map buttons here
