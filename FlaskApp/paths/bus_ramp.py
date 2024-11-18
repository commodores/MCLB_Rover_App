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

    ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:
        print("-- Creating a waypoint")

        the_connection.mav.mission_item_send(the_connection.target_system, # Target System: The ID of the target system (e.g., drone or UAV).
                                             the_connection.target_component, #Target Component: The ID of the target component within the system (e.g., flight controller, gimbal).
                                             waypoint.seq, #Sequence:  Ensure each waypoint has a unique sequence number (0, 1, 2, etc.).
                                             waypoint.frame, #Frame. Specifies the coordinate frame of the waypoint (e.g., global, local).
                                             waypoint.command, #Command:  The command associated with the waypoint (e.g., navigate to, loiter, takeoff).
                                             waypoint.current, #Current: : Indicates if this waypoint is the current active waypoint (1 for true, 0 for false).
                                             waypoint.auto,  #Auto Continue: determines if the mission should automatically continue to the next waypoint (1 for true, 0 for false).
                                             waypoint.param1,   #Used for various purposes, depending on the command (e.g., time to stay at the waypoint for loiter commands)
                                             waypoint.param2, #Acceptance Radius:  Acceptance radius in meters. This defines how close the drone must get to the waypoint to consider it reached.
                                             waypoint.param3, #pass radius: Pass radius in meters. Specifies the distance the vehicle can be from the waypoint when passing.
                                             waypoint.param4, #yaw :  Yaw angle at the waypoint (in degrees). If not required, set to NaN.
                                             waypoint.param5,#local x: The latitude of the waypoint (in degrees). Used when frame is MAV_FRAME_GLOBAL.
                                             waypoint.param6, #local y:t he longitude of the waypoint (in degrees). Used when frame is MAV_FRAME_GLOBAL.
                                             waypoint.param7, #locacl z: The altitude of the waypoint (in meters). The altitude can be absolute or relative based on the frame.
                                             waypoint.mission_type ) #missiont type: Specifies the mission type (e.g., normal mission, geofence).
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
    print("-- Message Read " + str(the_connection.recv_match(type=keyword, blocking =False)))



    # Main Function

    if __name__ == "__main__":
        print("-- Program Started")
        the_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
        the_connection.wait_heartbeat()

        while(the_connection.target_system == 0):
            print("-- Checking Heartbeat")
            the_connection.wait_heartbeat()
            print(" -- heatbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

        mission_waypoints = []

       # mission_waypoints.append(mission_item( 1, 3,	16,	0.00000000,	0.00000000,	0.00000000,	0.00000000,	31.55627590,	-84.17048360,	100.0000000, 1 ))
        #mission_waypoints.append(mission_item(2	,0,	3,	16,	0.00000000,	0.00000000,	0.00000000,	0.00000000,	31.55627140,	-84.17072500,	100.000000,	1))
        #ission_waypoints.append(mission_item(3, 0,	3,	16,	0.00000000,	0.00000000,	0.00000000,	0.00000000,	31.55626450,	-84.17096640,	100.000000,	1))
        #mission_waypoints.append(mission_item(4,0,	0,	3,	16,	0.00000000,	0.00000000,	0.00000000,	0.00000000,	31.55626450,	-84.17096700,	100.000000,	1))
        #mission_waypoints.append(mission_item(5,0,	0,	3,	16,	0.00000000,	0.00000000,	0.00000000,	0.00000000,	31.55627080,	-84.17072560,	100.000000,	1))
        #mission_waypoints.append(mission_item(6,0,	0,	3,	16,	0.00000000,	0.00000000,	0.00000000,	0.00000000,	31.55627650,	-84.17048360,	100.000000,	1))

        #missionwaypoint.append(mission_item( seq=1, current=0, x=31, y =-84, z=100.0))
        mission_waypoints.append(mission_item( 1, 1, 31.55627590,  84.17048360, 100.000000))
        mission_waypoints.append(mission_item( 2, 0, 31.55627140, -84.17072500,	100.000000))
        mission_waypoints.append(mission_item( 3, 0, 31.55626450, -84.17096640,	100.000000))
        mission_waypoints.append(mission_item( 4, 0, 31.55626450, -84.17096700,	100.000000))
        mission_waypoints.append(mission_item( 5, 0, 31.55627080, -84.17072560,	100.000000))
        mission_waypoints.append(mission_item( 6, 0, 31.55627650, -84.17048360,	100.000000))

        upload_misssion(the_connection, mission_waypoints)

        auto(the_connection)

        arm(the_connection)

        start_mission(the_connection)

        for mission_item in mission_waypoints:
            print("-- Message Read " + str(the_connection.recv_match(type="MISSION_ITEM_REACHED", condition= "MISSION_ITEM_REACHED.seq =={0}".format(mission_item.seq), blocking =True)))

        set_return(the_connection)



            
            













            
            





