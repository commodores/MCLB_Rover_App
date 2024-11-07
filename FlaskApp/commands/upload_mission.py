#Functions for submitting waypoints

import math
from pymavlink import mavutil

#Class for formatting the Mission Items

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
            self.param4 = math.nan 
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
def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("-- Sending Message out")

    the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, n, 0)


    for waypoint in mission_items:
        print("-- Creating a waypoint")

        the_connection.mav.mission_item_send(the_connection.target_system, the_connection.target_component, waypoint.seq, waypoint.frame, waypoint.command, waypoint.current, waypoint.auto, waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4, waypoint.param5,
                                                waypoint.param6,
                                                waypoint.param7,
                                                waypoint.mission_type)
        ack(the_connection, "COMMAND_ACK")


#send messages for the drone to return to launch point
def set_return(the_connection):

    print("-- Set Return to Launch")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_WAY_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0)
    
    ack(the_connection, "COMMAND_ACK")

#Start Mission func 
def start_mission(the_connection):
    print("--Mission Start")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_MISSION_START,0,0,0,0,0,0,0,0)
    
    ack(the_connection, "COMMAND_ACK")

#acknowledge func
def ack(the_connection, keyword):
    print("-- Message Read " + str(the_connection.recv_match(type=keyword, blocking = False)))



  # Example Function to upload mission

    if __name__ == "__main__":
        print("-- Program Started")
        the_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
        the_connection.wait_heartbeat()

        while(the_connection.target_system == 0):
            print("-- Checking Heartbeat")
            the_connection.wait_heartbeat()
            print(" -- heatbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

        mission_waypoints = []

        mission_waypoints.append(mission_item(0, 0, 31.5559748, -84.1694355, 0))
        #mission_waypoints.append(mission_item(1, 0, 42, -83, 0))
        #mission_waypoints.append(mission_item(2, 0, 42, -83, 0))

        upload_mission(the_connection, mission_waypoints)

        auto(the_connection)

        arm(the_connection)

        start_mission(the_connection)

        for mission_item in mission_waypoints:
            print("-- Message Read " + str(the_connection.recv_match(type="MISSION_ITEM_REACHED", condition= "MISSION_ITEM_REACHED.seq =={0}".format(mission_item.seq), blocking =True)))

        set_return(the_connection)



            
            


