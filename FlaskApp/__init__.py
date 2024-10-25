# Define the Flask app creation function and set up the mavlink_listener as a background task. This function will be responsible for emitting MAVLink messages to all connected clients.

import time
from flask import Flask

from .events import socketio
from .routes import main
from .rover_connect import rover_connection


def create_app():
    app = Flask(__name__)
    app.config["DEBUG"] = True
    app.config["SECRET_KEY"] = 'secret'

    app.register_blueprint(main)

    socketio.init_app(app, async_mode="eventlet")
    return app


# Background task to read MAVLink messages
def mavlink_listener():
    last_heartbeat_time = time.time()
    connected = False

    while True:
        # Check for heartbeat message
        msg = rover_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        print("Rover connected")
        
        if msg:
            # Update last heartbeat received time
            last_heartbeat_time = time.time()
            
            # If previously disconnected, now consider connected
            if not connected:
                connected = True
                socketio.emit("connection_status", {"status": "connected"})
        
        # If no heartbeat is received in the last 5 seconds, consider disconnected
        elif time.time() - last_heartbeat_time > 5:
            if connected:
                connected = False
                socketio.emit("connection_status", {"status": "disconnected"})
        
        # Emit MAVLink messages to clients if available
        msg = rover_connection.recv_match(blocking=False)
        if msg:
            socketio.emit("mavlink_message", msg.to_dict())