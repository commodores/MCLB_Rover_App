# Setting up SocketIO event handlers
# defines SocketIO event handlers for client interaction and command handling.

from pymavlink import mavutil
from .extensions import socketio, rover_connection


#
@socketio.on("connect")
def handle_connect():
    print("User Connected")

# Additional events for command handling can be added here, e.g., for controlling the rover

@socketio.on("send_command")
def handle_command(data):
    command = data.get("command")
    if command == "arm":
        # Arm the vehicle
        rover_connection.mav.command_long_send(
            rover_connection.target_system,
            rover_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        socketio.emit("status", {"message": "Vehicle armed"})
    elif command == "disarm":
        # Disarm the vehicle
        rover_connection.mav.command_long_send(
            rover_connection.target_system,
            rover_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        socketio.emit("status", {"message": "Vehicle disarmed"})
    else:
        socketio.emit("status", {"message": f"Unknown command: {command}"})

    