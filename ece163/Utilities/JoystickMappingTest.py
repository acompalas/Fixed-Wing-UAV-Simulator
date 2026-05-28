"""
Simple test utility that prints out the values of all axes and buttons of a 
detected controller to aid in setting up a controller. These will be used to
modify the ece163/Contants/JoystickConstants.py file as appropriate.

Requires pygame library to function (installed with bootstrap)
"""
# 4/18/2023 - Alex Swanson (adswanso@ucsc.edu) - Initial definition of file


#Allows this to find imports when run from ece163/ or from ece163/Utilities
import sys
sys.path.append('..')
sys.path.append('../..')


from ece163.Utilities.Joystick import Joystick
import pygame
import time

if __name__ == "__main__":
    print("Starting joystick test utility.\n")

    #Reused the controller class to initialize a joystick
    controller = Joystick()

    if not controller.active:
        #Print out the incompatible joysticks for testing purposes
        for stick_i in range(pygame.joystick.get_count()):
            print(pygame.joystick.Joystick(stick_i).get_name() + " found but not compatible.")

        exit()


    while(1):
        #Fetch the joystick movement
        pygame.event.get(pygame.JOYAXISMOTION)
        # pygame.event.get(pygame.BUTTON)

        #Print the output of each axis
        output = ""
        for i in range(controller.device.get_numaxes()):
            output += "Ax" + str(i) + ":" + "{:+.1f}".format(controller.device.get_axis(i)) +  " "

        output += " "

        for i in range(controller.device.get_numbuttons()):
            output += "Btn" + str(i) + ":" + "{:d}".format(controller.device.get_button(i)) +  " "

        print(output)

        #Sleep to prevent spam
        time.sleep(0.5)

    