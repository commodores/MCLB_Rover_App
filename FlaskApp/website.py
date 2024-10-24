from flask import Flask, logging, request, render_template, flash, session
import threading
import queue
from pymavlink import mavutil
import time
from flask import Flask, jsonify, redirect, url_for, render_template, request, flash, session
from pymavlink import mavutil
from datetime import timedelta
import math
from threading import Lock
import threading
from flask_socketio import SocketIO, emit


import serial
from paths.commands import arm_rover, disarm_rover, mission_reset, auto_mode, mission, reset_connection


app = Flask(__name__)

logging.basicConfig(level=logging.INFO)

# Initialize the connection and lock
the_connection= mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
serial_lock = threading.Lock()
command_queue = queue.Queue()  # Queue to hold commands for pymavlink thread


# Initialize key and session
app.secret_key = "hello"


# Function to run pymavlink in its own thread
def pymavlink_worker():
    while True:
        # Get the next command from the queue
        command = command_queue.get()
         # Log the received command
        if command == "arm":
            arm_rover()
             # Log success message
        elif command == "disarm":
            disarm_rover()
            
        elif command == "reset_mission":
            mission_reset()
        elif command == "reset_connection":
            reset_connection()
        elif command == "auto_mode":
            auto_mode()
        elif command == "warehouse":
            mission()
        # Add more commands as needed
        command_queue.task_done()


# Start the pymavlink thread
pymavlink_thread = threading.Thread(target=pymavlink_worker, daemon=True)
pymavlink_thread.start()


@app.route("/", methods=["POST", "GET"])
def home():
    global rover_status
    session.permanent = True


    if request.method == "POST":

        if "enable" in request.form:
            command_queue.put("arm")
            flash("Sent command to arm the rover", "info")
              
        elif "disable" in request.form:
            command_queue.put("disarm")

        elif "warehouse" in request.form:
            command_queue.put("mission")

        elif "reset_mission" in request.form:
            command_queue.put("reset_mission")

        elif "reset_connection" in request.form:
            command_queue.put("reset_connection")

        elif "auto_mode" in request.form:
            command_queue.put("auto_mode")

        elif "manual_mode" in request.form:
            command_queue.put("manual_mode")

        # Add more button logic here as needed


        return render_template("example.html")
   
    return render_template("example.html")


if __name__ == "__main__":
    app.run(debug=True)




