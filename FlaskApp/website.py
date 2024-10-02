from flask import Flask, redirect, url_for, render_template, request, flash, session
from pymavlink import mavutil 
from datetime import timedelta
import math
import threading
from paths.commands import arm_rover, disarm_rover, mission_reset, switch_modes, control_rover, create_new_connection, mission, mission_pause, manual_drive_mode
from paths.auto import connection, upload_misssion, auto,  start_mission, waypoints


app = Flask(__name__)
app.secret_key = "hello"
app.permanent_session_lifetime = timedelta(hours=5)


# New UI version of app
@app.route("/", methods=["POST", "GET"])
def home():

    session.permanent = True

    if request.method == "POST":

        if "enable" in request.form:
            arm_rover()
            flash("Rover is Enabled and Armed", "info")
            return render_template('example.html')
              
        if "disable" in request.form:
            disarm_rover()
            flash("Rover is Disable and Dismarmed", "info")
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
            manual_drive_mode()
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