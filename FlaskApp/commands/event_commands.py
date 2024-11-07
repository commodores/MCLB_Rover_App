#commands
# ARMING FUNCTIONS

import math
import sys
import time
from flask import flash
import pygame
from pymavlink import mavutil
from threading import Lock, Thread
from .upload_mission import auto, upload_mission, set_return, start_mission
from FlaskApp.extensions import socketio, rover_connection





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
        socketio.emit("status", {"message": "Rover DISARMED"})
        print("Rover DISARMED")
       


def reset_rover_connection():
    
    rover_connection.close

   # Create the connection
    rover_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    #time.sleep(2)
    # Wait a heartbeat before sending commands
    rover_connection.wait_heartbeat()

    print("New Connection Made")
    socketio.emit("messages", {"message": "New Connection Made"})



def auto_mode():
     # Wait a heartbeat before sending commands
    rover_connection.wait_heartbeat()
  
    # Choose a mode
    mode = 'AUTO'

    # Get mode ID
    mode_id = rover_connection.mode_mapping()[mode]

    # Set new mode
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

    # Set new mode
    rover_connection.mav.set_mode_send(
    rover_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
    print(f"Flight mode changed to {mode}")
    socketio.emit("messages", {"message": f"Rover is in {mode}"})



def stop_mission():
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

        print("Clearing all mission items")
        rover_connection.mav.mission_clear_all_send(rover_connection.target_system, rover_connection.target_component)

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

    upload_mission(rover_connection, mission_waypoints)

    print(" -- All Waypoints Created and Uploaded")
    socketio.emit("messages", {"message": "All Waypoints Created and Uploaded"})



def home_mission():

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

    upload_mission(rover_connection, mission_waypoints)

    print(" -- All Waypoints Created and Uploaded HOME MISSION")
    socketio.emit("messages", {"message": "All Waypoints Created and Uploaded"})



