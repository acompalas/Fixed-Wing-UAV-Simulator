"""
Holds the code to allow for the use of a joystick to drive the simulations as opposed to the current slider inputs.
This should all be relatively automatic, and require no tweaking from the students to make it work (caveat: you might need to reassign
the correct channels and buttons for your own particular joystick).

Also see ece163/Contants/JoystickConstants.py for the parameters that define the axes and buttons.
"""

# 4/18/2023 - Alex, Bailin, Gabe - first attempt at adding in the code
# 4/21/2023 - Documenting and folding into the main branch
# 5/02/2023 - Elkaim - All patched into code for 3-axis and 4-axis controllers, need to make sliders live for Chapter2



import pygame
from ..Containers.Inputs import joystickValues
from ..Constants import JoystickConstants as JSC


class Joystick:
    """
    Class which implements the joystick controls for the simulation. Should be fairly automatic for the students.
    Might need to re-map the axes and buttons for the joystick controller that you have. Mapping is held in the
    ece163/Contants/JoystickConstants.py file. Uses pygame library (installed automatically if you used bootstrap.py)
    """

    device = None

    def __init__(self):
        """
        Scans for valid joystick at startup. Note that this is only done once, and is not "live." Requires a minimum of
        three axes and 2 buttons to be valid.
        """
        pygame.joystick.init()

        #Search for a joystick with atleast 4 axises
        for stick_i in range(pygame.joystick.get_count()):
            self.device = pygame.joystick.Joystick(stick_i)
            if self.device.get_numaxes() < JSC.MIN_AXES or self.device.get_numbuttons() < JSC.MIN_BUTTONS:
                self.device.quit()
                self.device = None
            else:
                break

        #Check if a device connected
        if self.device == None:
            print("No compatible joysticks found, use the sliders.")
            self.active = False
        else:
            self.active = True
            #Display device name and number of axes
            print("Found " + self.device.get_name(), "\n\tAxes: " + str(self.device.get_numaxes()), "\n\tButtons: " + str(self.device.get_numbuttons()))

        pygame.init()

        

    def get_joystick_values(self):
        """
        Function to map the raw axis and button readings from the joystick to joystickValues type.

        :return: joystickValues class
        """
        pygame.event.get(pygame.JOYAXISMOTION)

        #Data is returned in a datatype to store relevant controller information
        if self.device.get_numaxes() > 3:
            controls = joystickValues(
                Throttle=   (self.device.get_axis(JSC.THROTTLE_AXIS)* (-1.0 if JSC.THROTTLE_REV else 1.0) + 1.0)/2.0, #The extra math scales this from 0-1
                Aileron=     self.device.get_axis(JSC.ROLL_AXIS)    * JSC.MAX_THROW * (-1.0 if JSC.ROLL_REV else 1.0),
                Elevator=    self.device.get_axis(JSC.PITCH_AXIS)   * JSC.MAX_THROW * (-1.0 if JSC.PITCH_REV else 1.0),
                Rudder=      self.device.get_axis(JSC.YAW_AXIS)     * JSC.MAX_THROW * (-1.0 if JSC.YAW_REV else 1.0),
                trigger=     self.device.get_button(JSC.TRIGGER_BUTTON),
                mode_button= self.device.get_button(JSC.MODE_CHANGE_BUTTON),

                #To add a custom axis or button, follow the following format. You can define constants in JoystickConstants.py
                #You will also need to add the field to the controllerValues type in the same file.
                
                # NewAxis=     self.device.get_axis(AXIS_INDEX)     * self.MAX_THROW * (-1.0 if JSC.AXIS_REV else 1.0)
            )
        else:
            controls = joystickValues(
                Throttle=   (self.device.get_axis(JSC.THROTTLE_AXIS)* (-1.0 if JSC.THROTTLE_REV else 1.0) + 1.0)/2.0, #The extra math scales this from 0-1
                Aileron=     self.device.get_axis(JSC.ROLL_AXIS)    * JSC.MAX_THROW * (-1.0 if JSC.ROLL_REV else 1.0),
                Elevator=    self.device.get_axis(JSC.PITCH_AXIS)   * JSC.MAX_THROW * (-1.0 if JSC.PITCH_REV else 1.0),
                Rudder=      0.0,
                trigger=     self.device.get_button(JSC.TRIGGER_BUTTON),
                mode_button= self.device.get_button(JSC.MODE_CHANGE_BUTTON),

                #To add a custom axis or button, follow the following format. You can define constants in JoystickConstants.py
                #You will also need to add the field to the controllerValues type in the same file.
                
                # NewAxis=     self.device.get_axis(AXIS_INDEX)     * self.MAX_THROW * (-1.0 if JSC.AXIS_REV else 1.0)
            )

        return controls