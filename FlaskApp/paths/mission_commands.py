#Mission commands
import math
import sys
import time
from flask import flash
import pygame
from pymavlink import mavutil
from threading import Lock, Thread

from FlaskApp.extensions import socketio, rover_connection
from FlaskApp.commands.event_commands import arm_rover
from .upload_mission import start_mission, upload_mission, auto


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

    
    auto(rover_connection)
    socketio.emit("messages", {"message": "Rover in"})

    arm_rover()
    socketio.emit("messages", {"message": "Rover is Armed"})

    start_mission(rover_connection)
    socketio.emit("messages", {"message": "Mission Started"})


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


def bus_ramp():
    
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

    mission_waypoints.append(mission_item( 1, 1, 31.55627590,  84.17048360, 100.000000))
    mission_waypoints.append(mission_item( 2, 0, 31.55627140, -84.17072500,	100.000000))
    mission_waypoints.append(mission_item( 3, 0, 31.55626450, -84.17096640,	100.000000))
    mission_waypoints.append(mission_item( 4, 0, 31.55626450, -84.17096700,	100.000000))
    mission_waypoints.append(mission_item( 5, 0, 31.55627080, -84.17072560,	100.000000))
    mission_waypoints.append(mission_item( 6, 0, 31.55627650, -84.17048360,	100.000000))

    upload_mission(rover_connection, mission_waypoints)

    print(" -- All Waypoints Created and Uploaded BUS RAMP MISSION")
    socketio.emit("messages", {"message": "All Waypoints BUS RAMP WAYPOINTS Created and Uploaded"})


def circle():

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

    mission_waypoints.append(mission_item( 1, 1, 31.55627590,  84.17048360, 100.000000))
    mission_waypoints.append(mission_item( 2, 0, 31.55627140, -84.17072500,	100.000000))
    mission_waypoints.append(mission_item( 3, 0, 31.55626450, -84.17096640,	100.000000))
    mission_waypoints.append(mission_item( 4, 0, 31.55626450, -84.17096700,	100.000000))
    mission_waypoints.append(mission_item( 5, 0, 31.55627080, -84.17072560,	100.000000))
    mission_waypoints.append(mission_item( 6, 0, 31.55627650, -84.17048360,	100.000000))

    upload_mission(rover_connection, mission_waypoints)

    print(" -- All Waypoints Created and Uploaded BUS RAMP MISSION")
    socketio.emit("messages", {"message": "All Waypoints BUS RAMP WAYPOINTS Created and Uploaded"})




def square():
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

    mission_waypoints.append(mission_item( 1, 1, 31.55627590,  84.17048360, 100.000000))
    mission_waypoints.append(mission_item( 2, 0, 31.55627140, -84.17072500,	100.000000))
    mission_waypoints.append(mission_item( 3, 0, 31.55626450, -84.17096640,	100.000000))
    mission_waypoints.append(mission_item( 4, 0, 31.55626450, -84.17096700,	100.000000))
    mission_waypoints.append(mission_item( 5, 0, 31.55627080, -84.17072560,	100.000000))
    mission_waypoints.append(mission_item( 6, 0, 31.55627650, -84.17048360,	100.000000))

    upload_mission(rover_connection, mission_waypoints)

    print(" -- All Waypoints Created and Uploaded BUS RAMP MISSION")
    socketio.emit("messages", {"message": "All Waypoints BUS RAMP WAYPOINTS Created and Uploaded"})




