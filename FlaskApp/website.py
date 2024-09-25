from flask import Flask, redirect, url_for, render_template, request, flash
from pymavlink import mavutil 
import math
from paths.commands import arm_rover, disarm_rover, mission_reset, switch_modes, mission


app = Flask(__name__)




@app.route("/", methods=["POST", "GET"])
def home():
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
          
        

# New UI version of app
@app.route("/ui", methods=["POST", "GET"])
def ui():
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
        if "switch_modes" in request.form: 
            switch_modes()
            return render_template("example.html")
        if "warehouse" in request.form:
            render_template("index.html") 
        return mission()
    else:
            return render_template("example.html")
    
@app.route('/print')
def printMsg():
    app.logger.warning('testing warning log')
    app.logger.error('testing error log')
    app.logger.info('testing info log')
    return "Check your console"

@app.route("/<usr>")
def user(usr):
    return f"<h1>{usr}</h1>"


if __name__ == "__main__":
    app.run(host='rover.local', port=5000, debug=True)