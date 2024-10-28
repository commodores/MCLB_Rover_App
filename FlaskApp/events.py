# Setting up SocketIO event handlers
# defines SocketIO event handlers for client interaction and command handling.
#Each event(soketio.on) is a thread

import math
import sys
import time
from flask import flash
import pygame
from pymavlink import mavutil
from .misson import auto, upload_misssion, set_return, start_mission
from .extensions import socketio, rover_connection


HOLD_MODE = 4 
@socketio.on("connect")
def handle_connect():
    print("User Connected")

# Additional events for command handling can be added here, e.g., for controlling the rover

@socketio.on("send_command")
def handle_command(data):
    command = data.get("command")
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
    #elif command == "mission":
     #   mission() #this works just trying smth
    elif command == "stop_mission":
        stop_mission()
    elif command == "reset_misson":
        reset_mission()
    else:
        socketio.emit("status", {"message": f"Unknown command: {command}"})

#Just trying smth
socketio.on("mission")
def handle_start_mission_command():
    mission()


#Just trying smth. Making single threads for mission and disable button
socketio.on("disable")
def handle_disable_command():
    disarm_rover()
    
# ARMING FUNCTIONS

def arm_rover():
    rover_connection.mav.command_long_send(
        rover_connection.target_system,
        rover_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    socketio.emit("status", {"message": "Rover armed"})
    print("Armed")


def disarm_rover():
    rover_connection.mav.command_long_send(
        rover_connection.target_system,
        rover_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    socketio.emit("status", {"message": "Rover disarmed"})
    print("disarm")


# CONNECTION FUNCTIONS
def reset_rover_connection():
    
    rover_connection.close

   # Create the connection
    rover_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    #time.sleep(2)
    # Wait a heartbeat before sending commands
    rover_connection.wait_heartbeat()

    print("New Connection Made")


# MODE FUNCTIONS

def auto_mode():
     # Wait a heartbeat before sending commands
    rover_connection.wait_heartbeat()
  

    # Choose a mode
    mode = 'AUTO'

    # Check if mode is available
    if mode not in rover_connection.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(rover_connection.mode_mapping().keys()))
        sys.exit(1)

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

    while True:
        # Wait for ACK command
        # Would be good to add mechanism to avoid endlessly blocking
        # if the autopilot sends a NACK or never receives the message
        ack_msg = rover_connection.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Continue waiting if the acknowledged command is not `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break


def manual_mode():
      # Choose a mode
    mode = 'MANUAL'

    # Get mode ID
    mode_id = rover_connection.mode_mapping()[mode]

    
    # Check if mode is available
    if mode not in rover_connection.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        flash('Unknown mode : {}'.format(mode))
        print('Try:', list(rover_connection.mode_mapping().keys()))
        sys.exit(1)
        flash('Try:', list(rover_connection.mode_mapping().keys()))
        sys.exit(1)

    rover_connection.mav.set_mode_send(
    rover_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

    print(f"Flight mode changed to {mode}")
    flash(f"Flight mode changed to {mode}")

def joystick2():
    pygame.init()
    pygame.joystick.init()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    rover_connection.wait_heartbeat()

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


# MISSION FUNCTIONS

def stop_mission():

    rover_connection.mav.set_mode_send(
        rover_connection.target_system,
        rover_connection.target_component,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        HOLD_MODE
    )


    # wait until disarming confirmed
    print("Waiting for the vehicle pause mission")
    flash("Waiting for the vehicle to pause mission")
    rover_connection.motors_disarmed_wait()
    print('Mission Paused!')
    flash("Mission Pause")


def reset_mission():

    rover_connection.mav.command_long_send(
        rover_connection.target_system,    # Target system (your vehicle)
        rover_connection.target_component, # Target component (usually MAV_COMP_ID_ALL)
        mavutil.mavlink.MAV_CMD_MISSION_CLEAR_ALL,  # Command ID for clearing all missions
        0,                                  # Confirmation (set to 0)
        0, 0, 0, 0, 0, 0                   # Parameters (not used for this command)
    )


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
    flash("Program Started")

    while(rover_connection.target_system == 0):
        print("-- Checking Heartbeat")
        flash("Checking Heartbeat")

        rover_connection.wait_heartbeat()
        print(" -- heatbeat from system (system %u component %u)" % (rover_connection.target_system, rover_connection.target_component))
        flash("heatbeat from system (system %u component %u)" % (rover_connection.target_system, rover_connection.target_component))

    mission_waypoints = []

    mission_waypoints.append(mission_item(0, 0, 59.000000, 1, 0))
    mission_waypoints.append(mission_item(1, 0, 31.55599420, -84.16967420, 0))
    mission_waypoints.append(mission_item(2, 0, 31.55608680, -84.16967350, 0))


    upload_misssion(rover_connection, mission_waypoints)
    print(" -- All Waypoints Created and Uploaded")

    auto(rover_connection)
    print("-- Rover put in Auto Mode")


    start_mission(rover_connection)
    print("-- Mission Started")

    for mission_item in mission_waypoints:
            print("-- Message Read " + str(rover_connection.recv_match(type="MISSION_ITEM_REACHED", condition= "MISSION_ITEM_REACHED.seq =={0}".format(mission_item.seq), blocking =True)))

    set_return(rover_connection)
    print(" -- Rover returning")



