import sys
import time
import math
import pygame
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
time.sleep(2)


def arm_rover():
    # Wait a heartbeat before sending commands
    the_connection.wait_heartbeat()

    # Arm
    # the_connection.arducopter_arm() or:
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with the_connection.motors_armed())
    print("Waiting for the vehicle to arm")
    the_connection.motors_armed_wait()
    print('Armed!')
    #msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    #print(msg)

def disarm_rover():
    # Wait a heartbeat before sending commands
    the_connection.wait_heartbeat()
    

    # Disarm
    # the_connection.arducopter_disarm() or:
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
