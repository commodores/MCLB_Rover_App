import time
from flask import Flask, jsonify, redirect, url_for, render_template, request, flash, session
from pymavlink import mavutil 
from datetime import timedelta
import math
from threading import Lock
import threading

import serial
from paths.commands import arm_rover, disarm_rover, mission_reset, switch_modes, control_rover, create_new_connection, mission, manual_drive_mode, joystick2


app = Flask(__name__)

# Initialize mavlink connection and a lock
ser = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
serial_lock = threading.Lock()

# Initialize key and session
app.secret_key = "hello"
app.permanent_session_lifetime = timedelta(hours=5)

#Initialize Lock
lock = Lock()

# Flag to control the mission
mission_running = False

# Function to arm the rover
def arm_rover_two():
    with serial_lock:
        try:
            ser.wait_heartbeat(timeout=5)
            ser.mav.command_long_send(
                ser.target_system, ser.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
        except Exception as e:
            flash(f"Failed to arm the rover: {e}", "error")

# Function to disarm the rover
def disarm_rover_two():
    with serial_lock:
        try:
            ser.wait_heartbeat(timeout=5)
            ser.mav.command_long_send(
                ser.target_system, ser.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0)
            global mission_running
            mission_running = False  # Stop any running mission
        except Exception as e:
            flash(f"Failed to disarm the rover: {e}", "error")

# Function to handle the mission
def mission():
    global mission_running
    mission_running = True
    while mission_running:
        with serial_lock:
            try:
                # Simulate sending waypoints or commands to the rover
                flash("Rover is continuing the mission...", "info")
                time.sleep(2)  # Example delay between mission steps
            except Exception as e:
                flash(f"Error during mission: {e}", "error")
                mission_running = False
        if not mission_running:
            break  # Exit the loop if mission was stopped

# New UI version of app
@app.route("/", methods=["POST", "GET"])
def home():

    session.permanent = True

    with lock:
        if request.method == "POST":

            if "enable" in request.form:
                arm_rover()
                return render_template('example.html')
                
            if "disable" in request.form:
                disarm_rover()
                return render_template("example.html")
            
            if "reset_path" in request.form:
                mission_reset()
                flash("Mission is reset", "info")
                return render_template("example.html")
            
            if "switch_modes" in request.form: 
                switch_modes()
                flash("Rover Mode is Switched", "info")
                return render_template("example.html")
            
            if "manual_control" in request.form: 
                control_rover()
                flash("Rover Mode is in Manual Control", "info")
                return render_template("example.html")
            
            if "reset_connection" in request.form: 
                create_new_connection()
                flash("Connection is reset", "info")
                return render_template("example.html")
            
            if "manual_drive" in request.form: 
                joystick2()
                flash("Flight mode changed to MANUAL", "info")
                return render_template("example.html")
            
            if "warehouse" in request.form:
                flash("Rover is going to Warehouse . . .", "info")
            mission()
            
            while "warehouse" in request.form:
                mission()
                # If "disable" is in the form, stop the mission
                if "disable" in request.form:
                    flash("Mission stopped.", "info")
                    break

                # Continue the mission
                
            return render_template("example.html")

    


            #upload_misssion(connection(), waypoints())
            #auto(connection())
            #start_mission(connection())
                #threading.Thread(target=mission).start()
                #mission()
            #flash("Rover is at Warehouse")
            #return render_template("example.html")
            
        # flash("No valid action was performed", "info")
            #return render_template("example.html")
    
        else:
                return render_template("example.html")
    pass
    return 'Request processed', 200
    

@app.route("/old", methods=["POST", "GET"])
def old():
    if request.method == "POST":
        if "enable" in request.form:
            flash("Armed")
            arm_rover()
            return render_template('index.html', flash("Armed"))      
        elif "disable" in request.form:
            disarm_rover()
            return render_template("index.html", armed = "Disabled")
        elif "reset_path" in request.form:
            mission_reset()
            return render_template("index.html", armed = "Disabled")
        elif "warehouse" in request.form:
            render_template("index.html") 
        return mission()
        
    else:
            return render_template("index.html")
    
#@app.route('/name', methods=["POST", "GET"])


#@app.route("/<usr>")
#def user(usr):
 #   return f"<h1>{usr}</h1>"


if __name__ == "__main__":
    app.run(host='rover.local', port=5000, debug=True)