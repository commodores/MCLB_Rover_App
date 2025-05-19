# defines Flask routes and pages

from flask import Blueprint, render_template

main = Blueprint("main", __name__)

@main.route("/")
def index():
    return render_template("home.html")

@main.route("/home.html")
def home():
    return render_template("home.html")

@main.route("/help.html")
def help():
    return render_template("help.html")
