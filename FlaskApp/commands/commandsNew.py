# Import mavutil
from asyncio import open_connection
import sys
import time
import math
import pygame
from pymavlink import mavutil
import serial
from auto import auto, upload_misssion, set_return, start_mission
from commands.arm import arm_rover

import os
import sys
import fcntl
from pymavlink import mavutil
import time
import tkinter as tk

LOCK_FILE = '/tmp/mavlink_lock.lock'  # Define the lock file path

def acquire_lock(lock_file):
    """Acquire a file lock."""
    fd = os.open(lock_file, os.O_CREAT | os.O_RDWR)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)  # Non-blocking lock
        return fd  # Return the file descriptor for the lock
    except BlockingIOError:
        print("Another instance is already running.")
        sys.exit(1)

def release_lock(fd):
    """Release the file lock."""
    fcntl.flock(fd, fcntl.LOCK_UN)
    os.close(fd)

class MAVLinkConnection:
    def __init__(self, master):
        self.master = master
        self.master.title("MAVLink Connection")
        
        # Connect button
        self.connect_button = tk.Button(master, text="Connect", command=self.connect)
        self.connect_button.pack(pady=10)
        
        # Heartbeat button
        self.heartbeat_button = tk.Button(master, text="Wait for Heartbeat", command=self.wait_for_heartbeat, state=tk.DISABLED)
        self.heartbeat_button.pack(pady=10)

        self.connection = None  # Initialize connection variable

    def connect(self):
        """Connect to the MAVLink serial port."""
        self.connection = mavutil.mavlink_connection('serial:/dev/ttyUSB0:57600')
        print("Connection established.")
      

    def wait_for_heartbeat(self):
        """Wait for a heartbeat and print a message."""
        if self.connection:
            print("Waiting for heartbeat...")
            self.connection.wait_heartbeat()
            print("Heartbeat received!")

def main():
    # Acquire the lock
    lock_fd = acquire_lock(LOCK_FILE)

    try:
        # Initialize Tkinter window
        root = tk.Tk()
        app = MAVLinkApp(root)
        
        # Start the Tkinter main loop
        root.mainloop()

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Cleanup
        if app and app.connection:
            app.connection.close()  # Ensure the connection is closed
        release_lock(lock_fd)  # Release the lock

if __name__ == "__main__":
    main()
