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

from .extensions import socketio
from .commands.event_commands import arm_rover, disarm_rover, reset_mission, reset_rover_connection, manual_mode, auto_mode, mission, stop_mission, home_mission


#Global variables
disabled_triggered = False
lock = Lock()

HOLD_MODE = 4 

# Handles connecting to app
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
    if command =="reset_rover_connection":
        reset_rover_connection()
    if command == "auto_mode":
        auto_mode()
    if command == "manual_mode":
        manual_mode()
    if command == "warehouse":
        mission()
    if command == "home":
        home_mission()
    if command == "stop_mission":
        stop_mission()
    if command == "reset_mission":
        reset_mission()
    



