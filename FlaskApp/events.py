# Setting up SocketIO event handlers
# defines SocketIO event handlers for client interaction and command handling.
#Each event(soketio.on) is a thread

import math
import sys
import time
from flask import flash
import pygame
from pymavlink import mavutil
from threading import Lock, Thread
from .misson import auto, upload_misssion, set_return, start_mission
from .extensions import socketio, rover_connection


#Global var
disabled_triggered = False
lock = Lock()

HOLD_MODE = 4 

@socketio.on("connect")
def handle_connect():
    print("User Connected")

# Additional events for command handling can be added here, e.g., for controlling the rover

@socketio.on("send_command")
def handle_command(data):
    global disabled_triggered

    command = data.get("command")

    with lock:
            if disabled_triggered:
                
                if command == "disable":
                    disabled_triggered = True
                    disarm_rover()
                elif command == "enable":
                    disabled_triggered = False
                    arm_rover()
                else:
                   disabled_triggered = False
                  # print("Disable priority: Ignoring command", command)
                   #socketio.emit("messages", {"message": f"Disable priority: Ignoring command{command}"})
                return
                        
    if command == "disable":
            disabled_triggered = True
            disarm_rover()
            return

    if command == "enable":
        arm_rover()
    #elif command == "disable":
        #   disarm_rover() #This works
    elif command =="reset_rover_connection":
        reset_rover_connection()
    elif command == "auto_mode":
        auto_mode()
    elif command == "manual_mode":
        manual_mode()
    elif command == "mission":
        mission() #this works just trying smth
    elif command == "stop_mission":
        stop_mission()
    elif command == "reset_misson":
        reset_mission()
    else:
        socketio.emit("messages", {"message": f"Unknown command: {command}"})



#Disable Functions

        
    
# ARMING FUNCTIONS

def disable_rover():
    print("Disabling rover...")
    disarm_rover()  # First, disarm the rover to halt any movement
    time.sleep(0.2)  # Brief pause to ensure disarm command registers
    stop_mission()   # Then clear any ongoing mission commands

def arm_rover():
    rover_connection.mav.command_long_send(
        rover_connection.target_system,
        rover_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    socketio.emit("status", {"message": "Rover ARMED"})
    print("Armed")
    
    
   



def disarm_rover():
     if rover_connection:
        # Command to disarm the rover
        rover_connection.mav.command_long_send(
            rover_connection.target_system,
            rover_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            0,  # Disarm (set to 1 to arm)
            0, 0, 0, 0, 0, 0  # Additional parameters are not used here
        )
        socketio.emit("status", {"message": "Rover Disarmed"})
        print("Rover DISARMED")
       

# CONNECTION FUNCTIONS
def reset_rover_connection():
    
    rover_connection.close

   # Create the connection
    rover_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    #time.sleep(2)
    # Wait a heartbeat before sending commands
    rover_connection.wait_heartbeat()

    print("New Connection Made")
    socketio.emit("messages", {"message": "New Connection is Made"})


# MODE FUNCTIONS

def auto_mode():
     # Wait a heartbeat before sending commands
    rover_connection.wait_heartbeat()
  

    # Choose a mode
    mode = 'AUTO'


    # Get mode ID
    mode_id = rover_connection.mode_mapping()[mode]
    # Set new mode
    # rover_connection.mav.command_long_send(
    #    rover_connection.target_system, rover_connection.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #    0, mode_id, 0, 0, 0, 0, 0) or:
    # rover_connection.set_mode(mode_id) or:
    rover_connection.mav.set_mode_send(
        rover_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    print(f"Flight mode changed to {mode}")
    socketio.emit("messages", {"message": f"Rover is in {mode}"})

   



def manual_mode():
      # Choose a mode
    mode = 'MANUAL'

    # Get mode ID
    mode_id = rover_connection.mode_mapping()[mode]


    rover_connection.mav.set_mode_send(
    rover_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

    print(f"Flight mode changed to {mode}")
    socketio.emit("messages", {"message": f"Rover is in {mode}"})

    # Initilze pygame and joystics

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

   
        
    print(f"Sending Manual Control - Roll: {roll}, Pitch: {pitch}, Throttle: {throttle}, Yaw: {yaw}")
    def send_manual_control(roll, pitch, throttle, yaw, buttons):
        rover_connection.mav.manual_control_send(
            rover_connection.target_system,    # Target system (Pixhawk)
            int(roll * 1000),        # X-axis (roll)
            int(pitch * 1000),       # Y-axis (pitch)
            int(throttle * 1000),    # Z-axis (throttle)
            int(yaw * 1000),         # R-axis (yaw)
            buttons                  # Buttons bitmask
        )

    DEADBAND = 0.9

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
        
        if abs(throttle) > DEADBAND :
           throttle = 0
        if abs(yaw)  > DEADBAND:
            yaw = 0
        if abs(roll)  > DEADBAND:
            roll = 0
        if abs(pitch)  > DEADBAND:
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

        
               
    
        
    

# MISSION FUNCTIONS

def stop_mission():
   #Works

   if rover_connection:
        # Set rover to MANUAL mode to override any autonomous actions
        rover_connection.mav.set_mode_send(
            rover_connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0  # Custom mode for MANUAL; adjust based on your vehicle setup
        )
        socketio.emit("messages", {"message": "Mission is Paused"})
        print("mission is paused and stopped")


       


def reset_mission():

    if rover_connection:

        print("Starting long mav send")
        rover_connection.mav.command_long_send(

            rover_connection.target_system, 
            print("found vehical"),   # Target system (your vehicle)
            rover_connection.target_component,
            print("found componenet"), # Target component (usually MAV_COMP_ID_ALL)
            mavutil.mavlink.MAV_CMD_MISSION_CLEAR_ALL,  # Command ID for clearing all missions
            0,                                  # Confirmation (set to 0)
            0, 0, 0, 0, 0, 0                   # Parameters (not used for this command)
        )
        socketio.emit("messages", {"message": "Mission is Reset"})
        print("mission is reset.")


def mission():
    class mission_item:
        def __init__(self, seq, current, x, y, z):
            self.seq = seq
            self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            self.current = current
            self.auto = 1
            self.param1 = 0.0
            self.param2 = 2.00
            self.param3 = 20.00
            self.param4 = math.nan #idk
            self.param5 = x
            self.param6 = y 
            self.param7 = z
            self.mission_type = 0

 

    print("-- Program Started")
    socketio.emit("messages", {"message": "Program Started"})

    while(rover_connection.target_system == 0):
        print("-- Checking Heartbeat")
        socketio.emit("messages", {"message": "Checking Heart Beat"})

        rover_connection.wait_heartbeat()
        print(" -- heatbeat from system (system %u component %u)" % (rover_connection.target_system, rover_connection.target_component))
        socketio.emit("messages", {"message": f"heatbeat from system {rover_connection.target_system, rover_connection.target_component}"})

    socketio.emit("messages", {"message": "Creating Way Points"})
    mission_waypoints = []

    mission_waypoints.append(mission_item(0, 0, 59.000000, 1, 0))
    mission_waypoints.append(mission_item(1, 0, 31.55599420, -84.16967420, 0))
    mission_waypoints.append(mission_item(2, 0, 31.55608680, -84.16967350, 0))


    upload_misssion(rover_connection, mission_waypoints)
    print(" -- All Waypoints Created and Uploaded")
    socketio.emit("messages", {"message": "All Waypoints Created and Uploaded"})

    auto(rover_connection)
    print("-- Rover put in Auto Mode")
    socketio.emit("messages", {"message": "Rover is in Auto Mode"})


    start_mission(rover_connection)
    print("-- Mission Started")
    socketio.emit("messages", {"message": "Mission Started"})

    #for mission_item in mission_waypoints:
            #print("-- Message Read " + str(rover_connection.recv_match(type="MISSION_ITEM_REACHED", condition= "MISSION_ITEM_REACHED.seq =={0}".format(mission_item.seq))))
            #socketio.emit("messages", {"message": f"{rover_connection.recv_match(type="MISSION_ITEM_REACHED", condition = "MISSION_ITEM_REACHED =={0}".format(mission_item.seq))}"})

    set_return(rover_connection)
    print(" -- Rover returning")
    socketio.emit("messages", {"message": "Rover is Returning to Set Position"})



