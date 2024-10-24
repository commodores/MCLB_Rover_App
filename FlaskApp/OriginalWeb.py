import time
from flask import Flask, jsonify, redirect, url_for, render_template, request, flash, session
from pymavlink import mavutil
from datetime import timedelta
import math
from threading import Lock
import threading


import serial
from paths.commands import arm_rover, disarm_rover, mission_reset, auto_mode, control_rover, reset_connection, mission, manual_drive_mode, joystick2




app = Flask(__name__)



# Initialize key and session
app.secret_key = "hello"
app.permanent_session_lifetime = timedelta(hours=5)





# New UI version of app
@app.route("/", methods=["POST", "GET"])
def home():


    session.permanent = True
    


    if request.method == "POST":


        if "enable" in request.form:
            arm_rover()
            return render_template('example.html')
            
        if "disable" in request.form:
            disarm_rover()
            return render_template("example.html")
        
        if "reset_path" in request.form:
            mission_reset()
            return render_template("example.html")
        
        if "auto_mode" in request.form:
            auto_mode()
            return render_template("example.html")
        
        if "manual_control" in request.form:
            control_rover()
            return render_template("example.html")
        
        if "reset_connection" in request.form:
            reset_connection()
            return render_template("example.html")
        
        if "manual_mode" in request.form:
            joystick2()
            return render_template("example.html")
        
        if "warehouse" in request.form:
            render_template("example.html")
        return mission()

    else:
           
            return render_template("example.html")




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

