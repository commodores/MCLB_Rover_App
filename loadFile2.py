from pymavlink import mavutil

# Connect to the vehicle
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master.wait_heartbeat()

# Function to send a waypoint
def send_waypoint(master, seq, frame, command, current, autocontinue, x, y, z):
    master.mav.mission_item_int_send(
        master.target_system, master.target_component,
        seq, frame, command, current, autocontinue, 0, 0, 0, 0,
        int(x * 1e7), int(y * 1e7), z)

# Set to AUTO mode to start the mission
def start_mission():
    master.set_mode_auto()

# Set the mode to GUIDED (or any other mission planning mode)
#master.set_mode_guided()

# Upload a simple mission with one waypoint
mission_count = 1
send_waypoint(master, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
              0, 1, -35.363261, 149.165230, 10)

# Send mission count to vehicle
master.mav.mission_count_send(master.target_system, master.target_component, mission_count)

# Wait for the vehicle to request waypoints and send them
for i in range(mission_count):
    msg = master.recv_match(type='MISSION_REQUEST', blocking=True)
    send_waypoint(master, i, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                  0, 1, -35.363261, 149.165230, 10)

# Start the mission
start_mission()

# Monitor the mission
while True:
    msg = master.recv_match(type=['MISSION_ITEM_REACHED', 'MISSION_CURRENT'], blocking=True)
    print(msg)
