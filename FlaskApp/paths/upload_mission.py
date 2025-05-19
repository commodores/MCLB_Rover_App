#Functions for submitting waypoints

from asyncio.log import logger
import math
import time
from pymavlink import mavutil
from FlaskApp.extensions import socketio, rover_connection



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
def arm (rover_connection):
    print("--Arming")
    rover_connection.mav.command_long_send(rover_connection.target_system, rover_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    ack(rover_connection, "COMMAND_ACK")
    socketio.emit("Rover is Armed")
   # ack(rover_connection, "COMMAND_ACK")
   

def auto(rover_connection):
    print("-- Seting Auto")
    rover_connection.set_mode_auto()
    ack(rover_connection, "COMMAND_ACK")
   
   # ack(rover_connection, "COMMAND_ACK")
   
def get_current_location(self, timeout=10):
        """Retrieves the current location of the drone."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            message = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if message:
                lat = message.lat / 1e7
                lon = message.lon / 1e7
                alt = message.alt / 1000.0
                logger.info(f"Current Location - Latitude: {lat}, Longitude: {lon}, Altitude: {alt} m")
                return lat, lon, alt
        raise TimeoutError("Failed to get current location")

 

   

#Upload Mission
def upload_mission(rover_connection, mission_items):
    n = len(mission_items)
    print("-- Sending Message out")

    rover_connection.mav.mission_count_send(rover_connection.target_system, rover_connection.target_component, n, 0)

    ack(rover_connection, "MISSION_REQUEST")


    for waypoint in mission_items:
        print("-- Creating a waypoint")

        rover_connection.mav.mission_item_send(rover_connection.target_system, rover_connection.target_component, waypoint.seq, waypoint.frame, waypoint.command, waypoint.current, waypoint.auto, waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4, waypoint.param5,
                                                waypoint.param6,
                                                waypoint.param7,
                                                waypoint.mission_type)
        ack(rover_connection, "MISSION_REQUEST")


#send messages for the drone to return to launch point
def set_return(rover_connection):

    print("-- Set Return to Launch")
    rover_connection.mav.command_long_send(rover_connection.target_system, rover_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_WAY_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0)
    
    ack(rover_connection, "COMMAND_ACK")

#Start Mission func 
def start_mission(rover_connection):
    print("--Mission Start")
    rover_connection.mav.command_long_send(rover_connection.target_system, rover_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_MISSION_START,0,0,0,0,0,0,0,0)
   
    ack(rover_connection, "COMMAND_ACK")

#acknowledge func
def ack(rover_connection, keyword):
    print("-- Message Read " + str(rover_connection.recv_match(type=keyword, blocking = False)))




  # Example Function to upload mission

    if __name__ == "__main__":
        print("-- Program Started")
        rover_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
        rover_connection.wait_heartbeat()

        while(rover_connection.target_system == 0):
            print("-- Checking Heartbeat")
            rover_connection.wait_heartbeat()
            print(" -- heatbeat from system (system %u component %u)" % (rover_connection.target_system, rover_connection.target_component))
            # Choose a mode
        mode = 'AUTO'

        # Get mode ID
        mode_id = rover_connection.mode_mapping()[mode]

        # Set new mode
        rover_connection.mav.set_mode_send(
            rover_connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        print(f"Flight mode changed to {mode}")
        socketio.emit("messages", {"message": f"Rover is in {mode}"})

        mission_waypoints = []

        mission_waypoints.append(mission_item(0, 0, 31.5559748, -84.1694355, 0))
        #mission_waypoints.append(mission_item(1, 0, 42, -83, 0))
        #mission_waypoints.append(mission_item(2, 0, 42, -83, 0))

        upload_mission(rover_connection, mission_waypoints)

        auto(rover_connection)

        arm(rover_connection)

        start_mission(rover_connection)

        for mission_item in mission_waypoints:
            print("-- Message Read " + str(rover_connection.recv_match(type="MISSION_ITEM_REACHED", condition= "MISSION_ITEM_REACHED.seq =={0}".format(mission_item.seq), blocking =True)))

        set_return(rover_connection)




            
            


