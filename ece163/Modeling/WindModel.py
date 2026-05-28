"""
Author: Anderson Compalas (acompala@ucsc.edu)

This file implements the WindModel class, which simulates atmospheric wind conditions affecting the aircraft.
It includes representations for steady wind components and stochastic turbulence using a Dryden gust model.
"""

import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC

isclose = lambda a, b: math.isclose(a, b, abs_tol=1e-12)

def safe_div(numerator, denominator, tol=1e-12):
    """ Perform division but handle 0/0 case and prevent division by exact zero. """
    if isclose(numerator, 0.0) and isclose(denominator, 0.0) or (numerator == 0.0 and denominator == 0.0):
        return 1.0  # Handle the special 0/0 case
    elif isclose(denominator, 0.0):
        return numerator / tol  # Prevent division by zero
    return numerator / denominator  # Normal division

def safe_exp(x, tol=709.78):
    """ Safe exponential to prevent overflow. If x is too large, cap. """
    return math.exp(min(x, tol))

class WindModel:
    def __init__(self, dT=VPC.dT, Va=VPC.InitialSpeed, drydenParameters=VPC.DrydenNoWind):
        """
        Initializes the WindModel class with default parameters.
        """
        self.dT = dT
        self.Va = Va
        self.drydenParameters = drydenParameters
        
        # Initialize state vectors for wind components
        self.xu = [[0]]
        self.xv = [[0], [0]]
        self.xw = [[0], [0]]
        
        # Initialize Dryden transfer function matrices
        self.Phi_u = [[0]]
        self.Gamma_u = [[0]]
        self.H_u = [[0]]  
        
        self.Phi_v = [[0, 0], [0, 0]]
        self.Gamma_v = [[0], [0]]
        self.H_v = [[0, 0]]  
        
        self.Phi_w = [[0, 0], [0, 0]]
        self.Gamma_w = [[0], [0]]
        self.H_w = [[0, 0]]  
        
        self.windState = States.windState()
        
        # Create the Dryden transfer functions
        self.CreateDrydenTransferFns(dT, Va, drydenParameters)
    
    def setWindModelParameters(self, Wn=0.0, We=0.0, Wd=0.0, drydenParameters=VPC.DrydenNoWind):
        """
        Sets the wind model parameters and updates the transfer functions.
        """
        self.windState.Wn = Wn
        self.windState.We = We
        self.windState.Wd = Wd
        self.drydenParameters = drydenParameters
        
        self.CreateDrydenTransferFns(self.dT, self.Va, drydenParameters)
    
    def reset(self):
        """
        Resets the wind model to default state.
        """
        self.xu = [[0]]
        self.xv = [[0], [0]]
        self.xw = [[0], [0]]
        
        self.Phi_u = [[0]]
        self.Gamma_u = [[0]]
        self.H_u = [[0]]  
        
        self.Phi_v = [[0, 0], [0, 0]]
        self.Gamma_v = [[0], [0]]
        self.H_v = [[0, 0]]  
        
        self.Phi_w = [[0, 0], [0, 0]]
        self.Gamma_w = [[0], [0]]
        self.H_w = [[0, 0]] 
        
        self.windState = States.windState()
        
        self.CreateDrydenTransferFns(self.dT, self.Va, self.drydenParameters)
    
    def setWind(self, windState):
        """
        Sets the wind state.
        """
        self.windState = windState
    
    def getWind(self):
        """
        Returns the current wind state.
        """
        return self.windState
    
    def getDrydenTransferFns(self):
        """
        Returns the Dryden transfer function matrices.
        """
        return (self.Phi_u, self.Gamma_u, self.H_u,
                self.Phi_v, self.Gamma_v, self.H_v,
                self.Phi_w, self.Gamma_w, self.H_w)
    
    def CreateDrydenTransferFns(self, dT, Va, drydenParameters):
        """
        Creates the Dryden transfer function matrices.
        """
        
        # Extract dryden parameters
        Lu, Lv, Lw = drydenParameters.Lu, drydenParameters.Lv, drydenParameters.Lw 
        sigma_u, sigma_v, sigma_w = drydenParameters.sigmau, drydenParameters.sigmav, drydenParameters.sigmaw
        
        # Hardcoded Values for zero initial conditions
        if isclose(Lu, 0.0) and isclose(Lv, 0.0) and isclose(Lw, 0.0) and \
            isclose(sigma_u, 0.0) and isclose(sigma_u, 0.0) and isclose(sigma_w, 0.0):
                self.Phi_u = [[1.0]]
                self.Gamma_u = [[0.0]]
                self.H_u = [[1.0]]

                self.Phi_v = [[1.0, 0.0], [0.0, 1.0]]
                self.Gamma_v = [[0.0], [0.0]]
                self.H_v = [[1.0, 1.0]]

                self.Phi_w = [[1.0, 0.0], [0.0, 1.0]]
                self.Gamma_w = [[0.0], [0.0]]
                self.H_w = [[1.0, 1.0]]

                return

        # U Coefficients
        exp_VaLu_dT = safe_exp(-safe_div(Va, Lu) * dT)
        self.Phi_u = [[exp_VaLu_dT]]
        self.Gamma_u = [[safe_div(Lu, Va) * (1 - exp_VaLu_dT)]]
        self.H_u = [[sigma_u * math.sqrt(2 / math.pi) * math.sqrt(safe_div(Va, Lu))]]  

        # V Coefficients
        exp_VaLv_dT = safe_exp(-safe_div(Va, Lv) * dT)
        self.Phi_v = [[exp_VaLv_dT * (1 - safe_div(Va, Lv) * dT), -exp_VaLv_dT * (safe_div(Va**2, Lv**2) * dT)],
                      [exp_VaLv_dT * dT, exp_VaLv_dT * (1 + safe_div(Va, Lv) * dT)]]
        self.Gamma_v = [[exp_VaLv_dT * dT],
                        [exp_VaLv_dT * (safe_div(Lv, Va) ** 2 * (safe_exp(safe_div(Va, Lv) * dT) - 1) - safe_div(Lv, Va) * dT)]]
        self.H_v = [[sigma_v * math.sqrt(3 / math.pi) * math.sqrt(safe_div(Va, Lv)), 
                     sigma_v * math.sqrt(3 / math.pi) * math.sqrt(safe_div(Va, Lv)) * safe_div(Va, math.sqrt(3) * Lv)]]  

        # W Coefficients
        exp_VaLw_dT = safe_exp(-safe_div(Va, Lw) * dT)
        self.Phi_w = [[exp_VaLw_dT * (1 - safe_div(Va, Lw) * dT), -exp_VaLw_dT * (safe_div(Va**2, Lw**2) * dT)],
                      [exp_VaLw_dT * dT, exp_VaLw_dT * (1 + safe_div(Va, Lw) * dT)]]
        self.Gamma_w = [[exp_VaLw_dT * dT],
                        [exp_VaLw_dT * (safe_div(Lw, Va) ** 2 * (safe_exp(safe_div(Va, Lw) * dT) - 1) - safe_div(Lw, Va) * dT)]]
        self.H_w = [[sigma_w * math.sqrt(3 / math.pi) * math.sqrt(safe_div(Va, Lw)), 
                     sigma_w * math.sqrt(3 / math.pi) * math.sqrt(safe_div(Va, Lw)) * safe_div(Va, math.sqrt(3) * Lw)]]


    def Update(self, uu=None, uv=None, uw=None):
        """
        Updates the wind state using the Dryden wind model.
        """
        uu = uu if uu is not None else random.gauss(0, 1)
        uv = uv if uv is not None else random.gauss(0, 1)
        uw = uw if uw is not None else random.gauss(0, 1)
        
        self.xu = [[self.Phi_u[0][0] * self.xu[0][0] + self.Gamma_u[0][0] * uu]]
        self.windState.Wu = self.H_u[0][0] * self.xu[0][0]
        
        self.xv = [[self.Phi_v[0][0] * self.xv[0][0] + self.Phi_v[0][1] * self.xv[1][0] + self.Gamma_v[0][0] * uv],
                   [self.Phi_v[1][0] * self.xv[0][0] + self.Phi_v[1][1] * self.xv[1][0] + self.Gamma_v[1][0] * uv]]
        self.windState.Wv = self.H_v[0][0] * self.xv[0][0] + self.H_v[0][1] * self.xv[1][0]
        
        self.xw = [[self.Phi_w[0][0] * self.xw[0][0] + self.Phi_w[0][1] * self.xw[1][0] + self.Gamma_w[0][0] * uw],
                   [self.Phi_w[1][0] * self.xw[0][0] + self.Phi_w[1][1] * self.xw[1][0] + self.Gamma_w[1][0] * uw]]
        self.windState.Ww = self.H_w[0][0] * self.xw[0][0] + self.H_w[0][1] * self.xw[1][0]
