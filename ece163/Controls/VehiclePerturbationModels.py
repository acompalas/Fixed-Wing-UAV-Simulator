"""
Author : Anderson Compalas (acompala@ucsc.edu)
This file contains the implementation of the VehiclePerturbationModels module, which defines functions for computing linearized flight dynamics models, including transfer function and state-space representations.

Functions included:
- dThrust_dVa: Computes the numerical derivative of thrust with respect to airspeed.
- dThrust_dThrottle: Computes the numerical derivative of thrust with respect to throttle.
- CreateTransferFunction: Generates the transfer function representation of the linearized flight model.
- CreateStateSpace: Converts the transfer function coefficients into a state-space representation.

Reference:
- Beard, Small Unmanned Aircraft: Theory and Practice (UAVBook), Chapter 5
- UAVBook Supplement for corrected equations
"""

import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Controls import VehicleTrim

def dThrust_dVa(Va, Throttle, epsilon=0.5):
    """ Computes the partial derivative of thrust with respect to airspeed numerically. """
    model = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    T_plus = model.CalculatePropForces(Va + epsilon, Throttle)[0]
    T_base = model.CalculatePropForces(Va, Throttle)[0]
    return (T_plus - T_base) / epsilon

def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    """ Computes the partial derivative of thrust with respect to throttle numerically. """
    model = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    T_plus = model.CalculatePropForces(Va, Throttle + epsilon)[0]
    T_base = model.CalculatePropForces(Va, Throttle)[0]
    return (T_plus - T_base) / epsilon

def CreateTransferFunction(trimState, trimInputs):
    Va = trimState.Va
    Throttle = trimInputs.Throttle

    # Compute Numerical Derivatives for Thrust Effects
    dT_dVa = dThrust_dVa(Va, Throttle)
    dT_dDeltaT = dThrust_dThrottle(Va, Throttle)

    # Define Transfer Function Terms (UAVbook_Supplement.pdf pg 26)
    a_V1 = (VPC.rho * Va * VPC.S / VPC.mass) * (VPC.CD0 + VPC.CDalpha * trimState.alpha + VPC.CDdeltaE * trimInputs.Elevator) - (dT_dVa / VPC.mass)
    a_V2 = dT_dDeltaT / VPC.mass
    a_V3 = VPC.g0 * math.cos(trimState.pitch - trimState.alpha)
    
    # Calculate Gamma
    V_body = [[trimState.u], [trimState.v], [trimState.w]]
    V_ned = MatrixMath.multiply(MatrixMath.transpose(trimState.R), V_body)

    Vn = V_ned[0][0]  # North velocity
    Ve = V_ned[1][0]  # East velocity
    Vd = V_ned[2][0]  # Down velocity

    # # Compute flight path angle (gamma)
    # gamma_trim = math.atan2(-Vd, math.sqrt(Vn**2 + Ve**2))
    # gamma_trim = math.atan2(Vd, trimState.chi)
    # Vg = math.sqrt(Vn**2 + Ve**2 + Vd**2)
    # gamma_trim = math.acos(trimState.chi/Vg)
    gamma_trim = trimState.pitch - trimState.alpha
    
    # Create Transfer Function Object
    tf = Linearized.transferFunctions()
    tf.Va_trim = Va           
    tf.alpha_trim = trimState.alpha
    tf.beta_trim = trimState.beta
    tf.gamma_trim = gamma_trim                                                    
    tf.theta_trim = trimState.pitch
    tf.phi_trim = trimState.roll
    tf.a_phi1 = -(1/2) * VPC.rho * Va**2 * VPC.S * VPC.b * VPC.Cpp * VPC.b/(2*Va)               # 5.23
    tf.a_phi2 = 1/2 * VPC.rho * Va**2 * VPC.S * VPC.b * VPC.CpdeltaA                            # 5.24
    tf.a_beta1 = -VPC.rho * Va * VPC.S * VPC.CYbeta / (2 * VPC.mass)                            # page 71 textbook
    tf.a_beta2 = VPC.rho * Va * VPC.S * VPC.CYdeltaR / (2 * VPC.mass)                           # page 71 textbook
    tf.a_theta1 = -VPC.rho * Va**2 * VPC.c * VPC.S * VPC.CMq * VPC.c / (2 * VPC.Jyy * 2 * Va)   # page 73 textbook
    tf.a_theta2 = -VPC.rho * Va**2 * VPC.c * VPC.S * VPC.CMalpha / (2 * VPC.Jyy)                # page 73 textbook
    tf.a_theta3 = VPC.rho * Va**2 * VPC.c * VPC.S * VPC.CMdeltaE / (2 * VPC.Jyy)                # page 73 textbook
    tf.a_V1, tf.a_V2, tf.a_V3 = a_V1, a_V2, a_V3

    return tf

def CreateStateSpace(trimState, trimInputs):
    """ Converts the transfer function coefficients into a state-space representation. """
    tf = CreateTransferFunction(trimState, trimInputs)

    # State-Space Representation
    ss = Linearized.stateSpace()
    ss.Va_trim = tf.Va_trim
    ss.alpha_trim = tf.alpha_trim
    ss.beta_trim = tf.beta_trim
    ss.gamma_trim = tf.gamma_trim
    ss.theta_trim = tf.theta_trim
    ss.phi_trim = tf.phi_trim

    ss.A_longitudinal = [[-tf.a_V1, 1, 0, 0, 0],
                         [-tf.a_theta2, -tf.a_theta1, 0, 0, 0],
                         [0, 1, 0, 0, 0],
                         [0, 0, 1, 0, 0],
                         [0, 0, 0, 1, 0]]

    ss.B_longitudinal = [[tf.a_V3, 0],
                         [tf.a_theta3, 0],
                         [0, 1],
                         [0, 0],
                         [0, 0]]

    ss.A_lateral = [[-tf.a_beta1, 1, 0, 0, 0],
                    [-tf.a_phi2, -tf.a_phi1, 0, 0, 0],
                    [0, 1, 0, 0, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 0, 1, 0]]

    ss.B_lateral = [[tf.a_beta2, 0],
                    [tf.a_phi2, 0],
                    [0, 1],
                    [0, 0],
                    [0, 0]]

    return ss
    