from pymavlink import mavutil

# Connect to the vehicle
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master.wait_heartbeat()

def set_mode(master, mode):
    # Get mode ID from mode name
    mode_id = master.mode_mapping()[mode]
    
    # Send the command to set the mode
    master.mav.command_long_send(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # Command to set mode
        0,  # Confirmation
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Mode flag
        mode_id,  # Custom mode
        0, 0, 0, 0, 0  # Unused parameters
    )

# Set the rover to AUTO mode
set_mode(master, 'AUTO')

# Confirm the mode change
msg = master.recv_match(type='HEARTBEAT', blocking=True)
print(f"Mode: {mavutil.mode_string_v10(msg)}, Armed: {msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED}")
