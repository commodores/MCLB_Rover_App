# This file initializes SocketIO and pymavlink connections.

from flask_socketio import SocketIO
from pymavlink import mavutil


# Create SocketIO instance
socketio = SocketIO()

# Set up the MAVLink connection 
rover_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)


