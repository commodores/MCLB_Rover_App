# Import mavutil
import math
from pymavlink import mavutil


# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()


#Class for formatting the Mission Ite,
class mission_item:
    def _init_(self,  current, x,y,z):
        self.seq = 1
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 2.00
        self.param3 = 20.00
        self.param4 = math.math
        self.param5 = x
        self.param6 = y 
        self.param7 = z
        self.mission_type = 0







