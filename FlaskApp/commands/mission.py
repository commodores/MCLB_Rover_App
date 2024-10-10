# Import mavutil
from pymavlink import mavutil

# Create the connection
the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

def mission_reset():
    the_connection.wait_heartbeat()
   

    print("sending commands")

    the_connection.mav.mission_clear_all_send(the_connection.target_system, the_connection.target_component)

   # the_connection.mav.command_long_send(
        #the_connection.target_system,
        #the_connection.target_component,
        #mavutil.mavlink.MISSION_RESET_DEFAULT,
        #the_connection.mav.mission_clear_all_send,
  #    ,
     #   0,
   #     0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    print('Mission Reset')
