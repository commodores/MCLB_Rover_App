 # Import mavutil
import sys
import time
import math
from typing import Iterable, Union
from pymavlink import mavutil


# Create the connection
the_connection = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
the_connection.wait_heartbeat()


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



AXIS_UNUSED = 0x7fff # INT16_MAX (manual control)

# from ardusub.com/developers/full-parameter-list.html#btnn-parameters
ARDUSUB_BTN_FUNCTIONS = {
    'disabled': 0,
    'shift': 1,
    'arm': 3,
    'disarm': 4,
    'mode_manual': 5,
    'mode_depth_hold': 7,
    'mount_center': 21,
    'mount_tilt_up': 22,
    'mount_tilt_down': 23,
    'lights1_brighter': 32,
    'lights1_dimmer': 33,
    'gain_inc': 42,
    'gain_dec': 43,
    # ... any others you're interested in
}

def set_param(autopilot, name: str, value: float, type: int,
              timeout: float=1):
    name = name.encode('utf8')
    autopilot.mav.param_set_send(
        autopilot.target_system, autopilot.target_component,
        name, value, type
    )
    
    msg = autopilot.recv_match(type='PARAM_VALUE', blocking=True,
                               timeout=timeout)
    # TODO: retries and/or verification that parameter is correctly set
    return msg

def set_button_function(autopilot, button: int, function: Union[int,str],
                        shifted=False):
    shifted = 'S' if shifted else ''
    param = f'BTN{button}_{shifted}FUNCTION'
    if isinstance(function, str):
        function = ARDUSUB_BTN_FUNCTIONS[function.lower()]
    type = mavutil.mavlink.MAV_PARAM_TYPE_INT8
    return set_param(autopilot, param, function, type)

def send_manual_control(autopilot, x=AXIS_UNUSED, y=AXIS_UNUSED, z=AXIS_UNUSED, 
                        r=AXIS_UNUSED, pressed_buttons: Union[int, Iterable[int]] = 0):
    ''' 
    'pressed_buttons' is either 
        a bit-field with 1 bits for pressed buttons (bit 0 -> button 0), or 
        an iterable specifying which buttons are pressed (e.g. [0,3,4])
    '''
    if not isinstance(pressed_buttons, int):
        # convert iterable into bit-field values
        pressed_buttons = sum(1 << button for button in pressed_buttons)
    autopilot.mav.manual_control_send(
        autopilot.target_system,
        x, y, z, r,
        pressed_buttons
    )

def press_release_buttons(autopilot, buttons):
    send_manual_control(autopilot, pressed_buttons=buttons)
    send_manual_control(autopilot, pressed_buttons=0)

autopilot = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
autopilot.wait_heartbeat()
print('autopilot connected!')

# assign button 2 to the 'shift' functionality
set_button_function(autopilot, 2, 'shift')
# assign button 1 to brighten lights1 on normal press, and dim them on shifted press
set_button_function(autopilot, 1, 'lights1_brighter')
set_button_function(autopilot, 1, 'lights1_dimmer', shifted=True)

press_release_buttons(autopilot, [1]) # make lights1 brighter
time.sleep(2) # wait a couple of seconds
press_release_buttons(autopilot, [1]) # make them another step brighter
time.sleep(2.5) # wait some more
press_release_buttons(autopilot, [1, 2]) # make them dimmer
time.sleep(1)
press_release_buttons(autopilot, [1, 2]) # dim once more