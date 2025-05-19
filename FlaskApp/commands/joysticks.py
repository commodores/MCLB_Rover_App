from pymavlink import mavutil
from commands import joystick
import pygame
# Set up the MAVLink connection 
rover_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

# Connect to Pixhawk
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for a heartbeat to establish communication
master.wait_heartbeat()

pygame.init()
pygame.joystick.init()

#joystick = []
joystick = pygame.joystick.Joystick(0)
joystick.init()



# Check the number of joysticks
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick detected.")
    sys.exit()



rover_connection.wait_heartbeat()

print("Joysticks Connected")
    # Function to send manual control to Pixhawk

   
        
    #print(f"Sending Manual Control - Roll: {roll}, Pitch: {pitch}, Throttle: {throttle}, Yaw: {yaw}")
def send_manual_control(roll, pitch, throttle, yaw, buttons):
        rover_connection.mav.manual_control_send(
            rover_connection.target_system,    # Target system (Pixhawk)
            int(roll * 1000),        # X-axis (roll)
            int(pitch * 1000),       # Y-axis (pitch)
            int(throttle * 1000),    # Z-axis (throttle)
            int(yaw * 1000),         # R-axis (yaw)
            buttons                  # Buttons bitmask
        )

DEADBAND = 0.1

# Send an initial zeroed control to ensure the motors are stopped


# Map joystick inputs to MAVLink control signals
while True:
    pygame.event.pump()

    roll, pitch, throttle, yaw = 0, 0, 0, 0

        #event handler
    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            # print(event)
            # pygame.joystick.Joystick(event.device_index)
            print("Joysticks Connected")
        if event.type == pygame.QUIT:
            pygame.quit()
  

         
        
        
        # Define a deadband threshold
        

        # Left joystick: Throttle and Yaw
        throttle = -joystick.get_axis(1)  # Y-axis (forward/backward) - Throttle
        yaw = joystick.get_axis(0)       # X-axis (left/right) - Yaw
        
        # Right joystick: Roll and Pitch
        roll = joystick.get_axis(3)      # X-axis (left/right) - Roll
        pitch = -joystick.get_axis(4)     # Y-axis (forward/backward) - Pitch
        
        if abs(throttle) < DEADBAND :
           throttle = 0
        if abs(yaw)  < DEADBAND:
            yaw = 0
        if abs(roll)  < DEADBAND:
            roll = 0
        if abs(pitch)  < DEADBAND:
            pitch = 0


        # Read button presses (optional)
        button_bitmask = 0
        if joystick.get_button(0):  # Button 0 for arming/disarming
            button_bitmask |= (1 << 0)
        
        # Send manual control to Pixhawk
        send_manual_control(roll, pitch, throttle, yaw, button_bitmask)

         # Debugging output
        print(f"Joystick Values - Throttle: {throttle}, Yaw: {yaw}, Roll: {roll}, Pitch: {pitch}, Button Bitmask: {button_bitmask}")

       # def handle_button_press():
            # Button 0: Arm the drone
        #    if joystick.get_button(0):
          #      print("Arming the Rover") 
         #       rover_connection.arducopter_arm()

            # Button 1: Disarm the drone
           # if joystick.get_button(1):
            #    print("Disarming the Rover")
             #   rover_connection.arducopter_disarm()

            # Button 2: Switch flight mode
            #if joystick.get_button(2):
             #   print("Switching to GUIDED mode")
              #  rover_connection.set_mode_px4('AUTO')

        # In the main loop:
        #handle_button_press()

        
               