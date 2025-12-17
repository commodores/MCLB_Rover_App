# 🤖 pymavlink_app
*The rover control web application for the MCLB Rover (HAZ Bot Project)*  
*Built using Flask and Socket.IO*

This application allows remote monitoring and control of the rover in real time, including mission uploading, telemetry display, and manual overrides via a clean web interface.

---

## 📑 Table of Contents
- [General Info](#general-info)
- [Technologies](#technologies)
- [Project Overview](#project-overview)
- [Setup](#setup)

---

## 🧠 General Info

The 'pymavlink_app' serves as a mission control web server for a MAVLink-compatible ground rover (e.g., running ArduPilot). The CubePilot Pixhawk connects to a Raspberry Pi via USB, establishing a MAVLink communication link. A touchscreen monitor is connected to the Raspberry Pi and embedded into the rover’s frame to enable onboard interaction.

- Arm and disarm the rover
- Upload and manage autonomous missions
- Monitor real-time telemetry
- Control the rover through a touchscreen interface

---

## ⚙️ Technologies

### Core Stack
- **OS:** Ubuntu Linux Desktop (running on Raspberry Pi 4)
- **Language:** Python 3
- **Web Framewrok:** Flask  
- **Real-Time Communication:** Flask-SocketIO (WebSockets)

### 🌐 Frontend

- **HTML / CSS / JavaScript** – UI styling and interactivity  
- **Jinja2** – Template rendering for Flask  
- **Socket.IO** – Used for live updates and telemetry push/pull

### 📡Mavlink Integration
- **Library** - pymavlink (Python implementation of MAVLink protocol)

**Sends commands such as**:
- Arming/disarming the rover
- Switching modes (AUTO, MANUAL, HOLD)
- Uploading and starting waypoint missions
- Receiving telemetry (GPS, battery, system status)
  
Managed in real time via a Python MAVLink connection (`mavutil`)

---


## 📁 Project Structure Overview

```plaintext
pymavlink_app/
├── .venv/                    # Virtual environment directory (excluded from version control)
├── .vscode/                  # Visual Studio Code settings and configurations
├── FlaskApp/                 # Main application package
│   ├── __init__.py           # Initializes Flask app and Socket.IO
│   ├── events.py             # Defines Socket.IO event handlers for real-time communication
│   ├── routes.py             # Manages HTTP routes and template rendering
│   ├── extensions.py         # Initializes Flask extensions (e.g., Socket.IO)
│   ├── rover_connect.py      # Establishes and manages MAVLink connection with the rover
│   ├── commands/             # Contains rover control command modules
│   │   └── event_commands.py # Handles commands like arming, mode switching, etc.
│   ├── paths/                # Mission planning and upload modules
│   │   ├── mission_commands.py  # Defines and executes mission plans
│   │   └── upload_mission.py    # Handles uploading and starting missions
│   ├── templates/            # HTML templates rendered by Flask
│   │   ├── home.html         # Main dashboard UI
│   │   └── help.html         # Help/documentation page for users
├── run.py                    # Entry point to launch the Flask server
└── README.md                 # Project documentation

---


## Setup

### 1. Power on the Rover
Turn on the rover using the breaker.

### 2. Boot the App
The app should start automatically.  
> **Note:** Currently, you need to log in manually. Automating startup will be implemented in a future update.

### 3. Open the Serial Port
To allow communication with the rover, open a terminal and run:

```bash
sudo chmod 666 /dev/ttyACM0


