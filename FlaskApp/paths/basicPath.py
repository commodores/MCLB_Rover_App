import sys
from pymavlink import mavutil
# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

while True:
    msg = master.recv_match(type = 'HEARTBEAT', blocking = False)
    if msg:
        mode = mavutil.mode_string_v10(msg)
        print(mode)                                                                                                                                                                                                                                                                                                                                                                            