# Import mavutil
from pymavlink import mavutil


# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()


master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, master.target_system,
                        master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(-35.3629849 * 10 ** 7), int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))


while 1:
    msg = master.recv_match(
        type='LOCAL_POSITION_NED', blocking=True)
    print(msg)