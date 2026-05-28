"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains two functions to get the control parameters from a closed loop system and vice versa.
"""

import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations

def computeTuningParameters(controlGains=Controls.controlGains(), linearizedModel=Linearized.transferFunctions()):
    try:
        tuningParameters = Controls.controlTuning()
        
        Vg = linearizedModel.Va_trim 
        K_thetaDC = (controlGains.kp_pitch * linearizedModel.a_theta3)/(linearizedModel.a_theta2 + (controlGains.kp_pitch * linearizedModel.a_theta3))
        
        # Roll Channel
        tuningParameters.Wn_roll = math.sqrt(controlGains.kp_roll * linearizedModel.a_phi2)
        tuningParameters.Zeta_roll = (linearizedModel.a_phi1 + linearizedModel.a_phi2 * controlGains.kd_roll) / (2 * tuningParameters.Wn_roll)
        
        # Course Channel
        tuningParameters.Wn_course = math.sqrt(controlGains.ki_course * VPC.g0/Vg)
        tuningParameters.Zeta_course = (VPC.g0 * controlGains.kp_course)/(2 * Vg  * tuningParameters.Wn_course)
        
        # Sideslip Channel
        tuningParameters.Wn_sideslip = math.sqrt(linearizedModel.a_beta2 * controlGains.ki_sideslip)
        tuningParameters.Zeta_sideslip = (linearizedModel.a_beta1 + (linearizedModel.a_beta2 * controlGains.kp_sideslip))/(2 * tuningParameters.Wn_sideslip)
        
        # Pitch Channel
        tuningParameters.Wn_pitch = math.sqrt(linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)
        tuningParameters.Zeta_pitch = (linearizedModel.a_theta1 + controlGains.kd_pitch * linearizedModel.a_theta3) / (2 * tuningParameters.Wn_pitch)
        
        # Altitude Channel
        tuningParameters.Wn_altitude = math.sqrt(K_thetaDC * linearizedModel.Va_trim * controlGains.ki_altitude)
        tuningParameters.Zeta_altitude = (K_thetaDC * linearizedModel.Va_trim * controlGains.kp_altitude)/(2 * tuningParameters.Wn_altitude)
        
        # Airspeed from Throttle
        tuningParameters.Wn_SpeedfromThrottle = math.sqrt(linearizedModel.a_V2 * controlGains.ki_SpeedfromThrottle)
        tuningParameters.Zeta_SpeedfromThrottle = (linearizedModel.a_V1 + (linearizedModel.a_V2 * controlGains.kp_SpeedfromThrottle))/ ( 2 * tuningParameters.Wn_SpeedfromThrottle)
        
        # Airspeed from Elevator
        tuningParameters.Wn_SpeedfromElevator = math.sqrt(-K_thetaDC * VPC.g0 * controlGains.ki_SpeedfromElevator)
        tuningParameters.Zeta_SpeedfromElevator = (linearizedModel.a_V1 - (K_thetaDC * VPC.g0 * controlGains.kp_SpeedfromElevator))/ (2 * tuningParameters.Wn_SpeedfromElevator)
        
        return tuningParameters
        
    except Exception:
        return Controls.controlTuning()

def computeGains(tuningParameters=Controls.controlTuning(), linearizedModel=Linearized.transferFunctions()):
    
    controlGains = Controls.controlGains()
    
    Vg = linearizedModel.Va_trim
    
    # Roll Channel
    controlGains.kp_roll = tuningParameters.Wn_roll ** 2 / linearizedModel.a_phi2
    controlGains.kd_roll = (2 * tuningParameters.Zeta_roll * tuningParameters.Wn_roll - linearizedModel.a_phi1) / linearizedModel.a_phi2
    controlGains.ki_roll = 0.001
    
    # Course Channel
    controlGains.kp_course = 2 * tuningParameters.Zeta_course * tuningParameters.Wn_course * Vg / VPC.g0
    controlGains.ki_course = tuningParameters.Wn_course ** 2 * Vg / VPC.g0
    
    # Sideslip Channel
    controlGains.kp_sideslip = (2 * tuningParameters.Zeta_sideslip * tuningParameters.Wn_sideslip - linearizedModel.a_beta1) / linearizedModel.a_beta2
    controlGains.ki_sideslip = tuningParameters.Wn_sideslip **2 / linearizedModel.a_beta2
    
    # Pitch Channel
    controlGains.kp_pitch = (tuningParameters.Wn_pitch ** 2 - linearizedModel.a_theta2) / linearizedModel.a_theta3
    controlGains.kd_pitch = (2 * tuningParameters.Zeta_pitch * tuningParameters.Wn_pitch - linearizedModel.a_theta1) / linearizedModel.a_theta3
    
    K_thetaDC = (controlGains.kp_pitch * linearizedModel.a_theta3)/(linearizedModel.a_theta2 + (controlGains.kp_pitch * linearizedModel.a_theta3))
    
    # Altitude Channel
    controlGains.ki_altitude = tuningParameters.Wn_altitude ** 2 / (K_thetaDC * Vg) # Va = Vg
    controlGains.kp_altitude = (2 * tuningParameters.Zeta_altitude * tuningParameters.Wn_altitude) / (K_thetaDC * Vg) # Va = Vg
    
    # Airspeed from Throttle
    controlGains.ki_SpeedfromThrottle = tuningParameters.Wn_SpeedfromThrottle ** 2 / linearizedModel.a_V2
    controlGains.kp_SpeedfromThrottle = (2 * tuningParameters.Zeta_SpeedfromThrottle * tuningParameters.Wn_SpeedfromThrottle - linearizedModel.a_V1) / linearizedModel.a_V2
    
    # Airspeed from Elevator
    controlGains.kp_SpeedfromElevator = (linearizedModel.a_V1 -  2 * tuningParameters.Zeta_SpeedfromElevator * tuningParameters.Wn_SpeedfromElevator) / (K_thetaDC * VPC.g0)
    controlGains.ki_SpeedfromElevator = -(tuningParameters.Wn_SpeedfromElevator**2/(K_thetaDC * VPC.g0))
    
    return controlGains
    


