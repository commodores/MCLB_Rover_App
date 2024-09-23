# Import mavutil
from pymavlink import mavutil

def arm_rover():
    # Create the connection
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    # Arm
    # master.arducopter_arm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')
    #msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    #print(msg)

def disarm_rover():
        # Create the connection
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    print("Waiting for the vehicle to dis arm")
    master.motors_armed_wait()
    print('Disarmed!')

def control_rover():
    # Create the connection
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    # Send a positive x value, negative y, negative z,
    # positive rotation and no button.
    # https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
    # Warning: Because of some legacy workaround, z will work between [0-1000]
    # where 0 is full reverse, 500 is no output and 1000 is full throttle.
    # x,y and r will be between [-1000 and 1000].
    master.mav.manual_control_send(
        master.target_system,
        500,
        -500,
        250,
        500,
        0)

    # To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
    # It's possible to check and configure this buttons in the Joystick menu of QGC
    buttons = 1 + 1 << 3 + 1 << 7
    master.mav.manual_control_send(
        master.target_system,
        0,
        0,
        500, # 500 means neutral throttle
        0,
        buttons)
    
def mission_reset():
    # Create the connection
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()

    print("sending commands")

    master.mav.mission_clear_all_send(master.target_system, master.target_component)

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



