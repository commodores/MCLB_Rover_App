"""
Example of how to use RC_CHANNEL_OVERRIDE messages to force input channels
in Ardupilot. These effectively replace the input channels (from joystick
or radio), NOT the output channels going to thrusters and servos.
"""

# Import mavutil
import sys
from pymavlink import mavutil

# Create the connection
the_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=9600) 

def joystick():
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
    # the_connection.mav.command_long_send(
    #    the_connection.target_system, the_connection.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #    0, mode_id, 0, 0, 0, 0, 0) or:
    # the_connection.set_mode(mode_id) or:
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



    # Create a function to send RC values
    # More information about Joystick channels
    # here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
    def set_rc_channel_pwm(channel_id, pwm=1500):
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        the_connection.mav.rc_channels_override_send(
            the_connection.target_system,                # target_system
            the_connection.target_component,             # target_component
            *rc_channel_values)                  # RC channel list, in microseconds.


    # Set some roll
    set_rc_channel_pwm(2, 1600)

    # Set some yaw
    set_rc_channel_pwm(4, 1600)

    set_rc_channel_pwm(4, 1600)

    # The camera pwm value sets the servo speed of a sweep from the current angle to
    #  the min/max camera angle. It does not set the servo position.
    # Set camera tilt to 45º (max) with full speed
    set_rc_channel_pwm(8, 1900)

    # Set channel 12 to 1500us
    # This can be used to control a device connected to a servo output by setting the
    # SERVO[N]_Function to RCIN12 (Where N is one of the PWM outputs)
    set_rc_channel_pwm(12, 1500)