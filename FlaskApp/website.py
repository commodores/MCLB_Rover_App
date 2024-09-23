from flask import Flask, redirect, url_for, render_template, request
from pymavlink import mavutil 
import math
from paths.commands import arm_rover, disarm_rover, mission_reset, control_rover
from paths.auto import mission, mission_item
from paths.path import mission2

app = Flask(__name__)

#the_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

@app.route("/", methods=["POST", "GET"])
def home():
    if request.method == "POST":
        if "enable" in request.form:
            arm_rover()
            return render_template('index.html', armed = "Armed")      
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
          
        

# New UI version of app
@app.route("/ui", methods=["POST", "GET"])
def login():
    if request.method == "POST":
        if "enable" in request.form:
            arm_rover()
            return render_template('example.html', armed = "Armed")      
        if "disable" in request.form:
            disarm_rover()
            return render_template("example.html", armed = "Disabled")
        if "reset_path" in request.form:
            mission_reset()
            return render_template("example.html", armed = "Disabled")
        if "manual_control" in request.form: 
            control_rover()
            return render_template("example.html")
        if "warehouse" in request.form: 
           return mission()
    else:
            return render_template("example.html")

@app.route("/<usr>")
def user(usr):
    return f"<h1>{usr}</h1>"


if __name__ == "__main__":
    app.run(host='rover.local', port=5000, debug=True)