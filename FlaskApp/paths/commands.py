# Import mavutil
import sys
import time
import math
from pymavlink import mavutil
from paths.auto import auto, upload_misssion, set_return, start_mission


 # Create the connection
the_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=9600) 
time.sleep(2)

def arm_rover():
    # Wait a heartbeat before sending commands
    the_connection.wait_heartbeat()


    # Arm
    # master.arducopter_arm() or:
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    action_messages = print("Waiting for the vehicle to arm")
    the_connection.motors_armed_wait()
    print('Armed!')
    #msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    #print(msg)

def disarm_rover():
    # Wait a heartbeat before sending commands
    the_connection.wait_heartbeat()
    

    # Disarm
    # master.arducopter_disarm() or:
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    print("Waiting for the vehicle to dis arm")
    the_connection.motors_armed_wait()
    print('Disarmed!')

def control_rover():
    # Wait a heartbeat before sending commands
    the_connection.wait_heartbeat()
    

    # Send a positive x value, negative y, negative z,
    # positive rotation and no button.
    # https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
    # Warning: Because of some legacy workaround, z will work between [0-1000]
    # where 0 is full reverse, 500 is no output and 1000 is full throttle.
    # x,y and r will be between [-1000 and 1000].
    the_connection.mav.manual_control_send(
        the_connection.target_system,
        500,
        -500,
        250,
        500,
        0)

    # To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
    # It's possible to check and configure this buttons in the Joystick menu of QGC
    buttons = 1 + 1 << 3 + 1 << 7
    the_connection.mav.manual_control_send(
        the_connection.target_system,
        0,
        0,
        500, # 500 means neutral throttle
        0,
        buttons)
    
def mission_reset():
    the_connection.wait_heartbeat()
   

    print("sending commands")

    the_connection.mav.mission_clear_all_send(the_connection.target_system, the_connection.target_component)

   # master.mav.command_long_send(
        #master.target_system,
        #master.target_component,
        #mavutil.mavlink.MISSION_RESET_DEFAULT,
        #master.mav.mission_clear_all_send,
  #    ,
     #   0,
   #     0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    print('Mission Reset')


def switch_modes():

    # Wait a heartbeat before sending commands
    the_connection.wait_heartbeat()
  

    # Choose a mode
    mode = 'AUTO'

    # Check if mode is available
    if mode not in the_connection.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(the_connection.mode_mapping().keys()))
        sys.exit(1)

    # Get mode ID
    mode_id = the_connection.mode_mapping()[mode]
    # Set new mode
    # master.mav.command_long_send(
    #    master.target_system, master.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #    0, mode_id, 0, 0, 0, 0, 0) or:
    # master.set_mode(mode_id) or:
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    while True:
        # Wait for ACK command
        # Would be good to add mechanism to avoid endlessly blocking
        # if the autopilot sends a NACK or never receives the message
        ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Continue waiting if the acknowledged command is not `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break


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



def create_new_connection():
    # Create the connection
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    time.sleep(2)
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    print("New Connection Made")

def mission_pause():
     # Wait a heartbeat before sending commands
    the_connection.wait_heartbeat()
    

# Send command to pause the mission (MAV_CMD_DO_PAUSE_CONTINUE)
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
        0, # Confirmation
        19,  # 19 pauses the mission (20 would continue the mission)
        0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    print("Waiting for the vehicle pause mission")
    the_connection.motors_disarmed_wait()
    print('Mission Paused!')

def manual_drive_mode():
    # Choose a mode
    mode = 'MANUAL'

    # Get mode ID
    mode_id = the_connection.mode_mapping()[mode]

    
    # Check if mode is available
    if mode not in the_connection.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(the_connection.mode_mapping().keys()))
        sys.exit(1)

    the_connection.mav.set_mode_send(
    the_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

    print(f"Flight mode changed to {mode}")

    