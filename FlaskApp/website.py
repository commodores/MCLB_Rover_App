from flask import Flask, redirect, url_for, render_template, request
from pymavlink import mavutil 
import math
from paths.commands import arm_rover, disarm_rover
from paths.auto import mission

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
                return render_template("index.html", armed = "Disable")
            elif "submit" in request.form:
                 render_template("index.html",  path = "Warehouse")  
                 mission()
                 return       
    else:
          return render_template("index.html")
          
        

# Other web page routes(Did in tutorial)
@app.route("/login", methods=["POST", "GET"])
def login():
    if request.method == "POST":
        user = request.form["nm"]
        return redirect(url_for("user", usr=user))
    else:
        return render_template("login.html")


@app.route("/<usr>")
def user(usr):
    return f"<h1>{usr}</h1>"


if __name__ == "__main__":
    app.run(host='rover.local', port=5000, debug=True)