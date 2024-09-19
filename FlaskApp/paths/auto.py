# Import mavutil
import math
from pymavlink import mavutil


# Create the connection
#master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
#master.wait_heartbeat()


#Class for formatting the Mission Ite,
class mission_item:
        def __init__(self, seq, current, x, y, z):
            self.seq = seq
            self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            self.current = current
            self.auto = 1
            self.param1 = 0.0
            self.param2 = 2.00
            self.param3 = 20.00
            self.param4 = math.nan #idk
            self.param5 = x
            self.param6 = y 
            self.param7 = z
            self.mission_type = 0

#Arm the Rover
def arm (the_connection):
    print("--Arming")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    ack(the_connection, "COMMAND_ACK")

def auto(the_connection):
    print("-- Seting Auto")
    the_connection.set_mode_auto()
    ack(the_connection, "COMMAND_ACK")

#Upload Mission
def upload_misssion(the_connection, mission_items):
    n = len(mission_items)
    print("-- Sending Message out")

    the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, n, 0)

    #ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:
        print("-- Creating a waypoint")

        the_connection.mav.mission_item_send(the_connection.target_system, the_connection.target_component, waypoint.seq, waypoint.frame, waypoint.command, waypoint.current, waypoint.auto, waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4, waypoint.param5,
                                                waypoint.param6,
                                                waypoint.param7,
                                                waypoint.mission_type)
    if waypoint !=mission_items[n-1]:
        the_connection.mav.mission_ack_send(the_connection, "MISSION_REQUEST")
        
    ack(the_connection, "MISSION_ACK")


#send messages for the drone to return to launch point
def set_return(the_connection):
    print("-- Set Return to Launch")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_WAY_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0)
    
    ack(the_connection, "COMMAND_ACK")

#Start Mission
def start_mission(the_connection):
    print("--Mission Start")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_MISSION_START,0,0,0,0,0,0,0,0)
    
    ack(the_connection, "COMMAND_ACK")

#acknowledge func
def ack(the_connection, keyword):
    print("-- Message Read " + str(the_connection.recv_match(type=keyword, blocking =True)))



# Main Function
#if __name__ == "__main__":
def mission():

    class mission_item:
        def __init__(self, seq, current, x, y, z):
            self.seq = seq
            self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            self.current = current
            self.auto = 1
            self.param1 = 0.0
            self.param2 = 2.00
            self.param3 = 20.00
            self.param4 = math.nan #idk
            self.param5 = x
            self.param6 = y 
            self.param7 = z
            self.mission_type = 0

    print("-- Program Started")
    the_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

    while(the_connection.target_system == 0):
        print("-- Checking Heartbeat")
        the_connection.wait_heartbeat()
        print(" -- heatbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

    mission_waypoints = []

    mission_waypoints.append(mission_item(0, 0, 59.000000, 1, 0))
    mission_waypoints.append(mission_item(1, 0, 31.55599420, -84.16967420, 0))
    mission_waypoints.append(mission_item(2, 0, 31.55608680, -84.16967350, 0))

    upload_misssion(the_connection, mission_waypoints)

    auto(the_connection)

    start_mission(the_connection)

    for mission_item in mission_waypoints:
        print("-- Message Read " + str(the_connection.recv_match(type="MISSION_ITEM_REACHED", condition= "MISSION_ITEM_REACHED.seq =={0}".format(mission_item.seq), blocking =True)))

    set_return(the_connection)















