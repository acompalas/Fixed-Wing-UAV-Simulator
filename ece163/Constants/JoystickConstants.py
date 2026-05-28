"""
This is a configuration file to map the joystick axes and buttons appropriately
so that these can be used for interfacing with the ece163 simulation in a way that
is slightly easier to handle than the slider inputs.

These settings can be checked using the Ultities->JoystickMappingTest.py which will
output the Axes values and button presses for the attached joystick. Change the values
below to match your inputs. Note that for 3-axes joysticks, we will leave out the yaw
input (use throttle instead); this is only relevant to Chapter4.py simulation.
"""

# 4/18/2023 - Alex Swanson (adswanso@ucsc.edu) - Initial definition of file
# 5/02/2023 - Elkaim - clean up 4-axis vs 3-axis code.
# 5/4/2023 - Elkaim and Swanson - Finished code testing




################################################################################
# These are for the Logitech EXTREME 3DPRO using the main stick for control    #
################################################################################
ROLL_AXIS = 0
PITCH_AXIS = 1
YAW_AXIS = 2
THROTTLE_AXIS = 3
ROLL_REV = 0
PITCH_REV = 1 # Pitch axis is reversed.
YAW_REV = 0
THROTTLE_REV = 1 # Throttle axis is reversed.
TRIGGER_BUTTON = 0 # Used to activate the controller
MODE_CHANGE_BUTTON = 1 # Used to switch between what is controlled

################################################################################
# These are for the Logitech F310 using the thumbsticks for control            #
################################################################################
# ROLL_AXIS = 2
# PITCH_AXIS = 3
# YAW_AXIS = 0
# THROTTLE_AXIS = 1
# ROLL_REV = 0
# PITCH_REV = 1 # Pitch axis is reversed.
# YAW_REV = 0
# THROTTLE_REV = 1 # Throttle axis is reversed.
# TRIGGER_BUTTON = 0 # Used to activate the controller
# MODE_CHANGE_BUTTON = 1 # Used to switch between what is controlled

#########################################################################################
# These are for the Logitech Wingman Attack-2 using the main stick for control (3-Axis) #
#########################################################################################
# ROLL_AXIS = 0
# PITCH_AXIS = 1
# YAW_AXIS = None # only 3 axes, leave yaw off.
# THROTTLE_AXIS = 2
# ROLL_REV = 0
# PITCH_REV = 1 # Pitch axis is reversed.
# YAW_REV = None # only 3 axes, leave yaw off
# THROTTLE_REV = 1 # Throttle axis is reversed.
# TRIGGER_BUTTON = 0 #Used to activate the controller
# MODE_CHANGE_BUTTON = 3 #Used to switch between what is controlled

#This defines the minimum number of axes a controller must have for it to be used with the sim
MIN_AXES = 3
MIN_BUTTONS = 2


#Chapter2.py constants
CHAPTER2_MAX_TRANSLATION = 10
CHAPTER2_MAX_ANGLE = 180

#Chapter3.py constants
CHAPTER3_MAX_FORCE = 10
CHAPTER3_MAX_MOMENT = 0.1

#Chapter4.py constants
CHAPTER4_MAX_THROW = 0.3


#Chapter6.py constants
CHAPTER6_MIN_AIRSPEED = 20
CHAPTER6_MAX_AIRSPEED = 30
CHAPTER6_MIN_ALTITUDE = 50
CHAPTER6_MAX_ALTITUDE = 250
CHAPTER6_MIN_COURSE = -180
CHAPTER6_MAX_COURSE = 180




# This controls the range of values that is output. If you want it to go from -1 to 1, 
# set it to 1. If you want -0.3 to 0.3, set it to 0.3
MAX_THROW = 1.0