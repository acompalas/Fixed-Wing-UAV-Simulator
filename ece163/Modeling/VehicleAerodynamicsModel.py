"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains a class for vehicle aerodynamics. Formulas sourced from Beard textbook and Dryden handout.
"""

import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

class VehicleAerodynamicsModel:
    def __init__(self, initialSpeed=VPC.InitialSpeed, initialHeight=-VPC.InitialDownPosition):
        """
        Initializes the Vehicle Aerodynamics Model.
        """
        self.vehicleDynamics = VehicleDynamicsModel.VehicleDynamicsModel()
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.windModel = WindModel.WindModel()
        
        # Initialize vehicle state
        self.reset()

    def reset(self):
        """
        Resets the aerodynamics model to its initial state.
        """
        self.vehicleDynamics.reset()
        
        # Restore initial speed and height
        self.vehicleDynamics.state.Va = self.initialSpeed
        self.vehicleDynamics.state.u = self.initialSpeed
        self.vehicleDynamics.state.pd = self.initialHeight 
        self.windModel.reset()

    def getVehicleState(self):
        """
        Returns the current vehicle state.
        """
        return self.vehicleDynamics.getVehicleState()

    def setVehicleState(self, new_state):
        """
        Sets the vehicle state.
        """
        self.vehicleDynamics.setVehicleState(new_state)

    def getVehicleDynamicsModel(self):
        """
        Returns the Vehicle Dynamics Model instance.
        """
        return self.vehicleDynamics
    
    def getWindModel(self):
        """
        Returns the current WindModel instance.
        """
        return self.windModel

    def setWindModel(self, windModel):
        """
        Sets the WindModel instance.
        """
        self.windModel = windModel

    def Update(self, controls):
        """
        Updates the state of the vehicle with the aerodynamics model.
        """
        state = self.getVehicleState()
        wind = self.windModel.getWind()
        forces_moments = self.updateForces(state, controls, wind)
        self.vehicleDynamics.Update(forces_moments)

    def gravityForces(self, state):
        """
        Computes the gravitational forces acting on the vehicle.
        """
        g_force = [0, 0, VPC.g0 * VPC.mass]  # Gravity in NED frame (positive down)
        gravity_body = MatrixMath.multiply(state.R, [[g_force[0]], [g_force[1]], [g_force[2]]]) # Rotate to body
        
        forces_moments = Inputs.forcesMoments()
        forces_moments.Fx = gravity_body[0][0]
        forces_moments.Fy = gravity_body[1][0]
        forces_moments.Fz = gravity_body[2][0]
        
        return forces_moments

    def CalculateCoeff_alpha(self, alpha):
        """
        Computes lift, drag, and moment coefficients based on angle of attack.
        """
        CL_attached = VPC.CL0 + VPC.CLalpha * alpha
        CD_attached = VPC.CDp + ((VPC.CL0 + VPC.CLalpha * alpha) ** 2) / (math.pi * VPC.AR * VPC.e)
        
        CL_separated = 2 * math.sin(alpha) * math.cos(alpha)
        CD_separated = 2 * math.sin(alpha) ** 2
        
        sigma = ((1 + math.exp(-VPC.M * (alpha - VPC.alpha0))) + math.exp(VPC.M * (alpha + VPC.alpha0))) / \
                ((1 + math.exp(-VPC.M * (alpha - VPC.alpha0))) * (1 + math.exp(VPC.M * (alpha + VPC.alpha0))))
        
        CL = (1 - sigma) * CL_attached + sigma * CL_separated
        CD = (1 - sigma) * CD_attached + sigma * CD_separated
        Cm = VPC.CM0 + VPC.CMalpha * alpha
        
        return CL, CD, Cm

    def aeroForces(self, state):
        """
        Computes aerodynamic forces and moments excluding control surfaces.
        """
        
        # Prevent Division by Zero
        eps = 1e-16 
        Va_safe = state.Va 
        
        if state.Va == 0:
            Va_safe = eps
        
        # Compute aerodynamic coefficients
        CL, CD, Cm = self.CalculateCoeff_alpha(state.alpha)
        qbar = 0.5 * VPC.rho * (Va_safe ** 2) * VPC.S

        # Initialize forces and moments
        forces_moments = Inputs.forcesMoments()

        # Compute Lift and Drag Forces (4.6 and 4.7 first 2 terms)
        F_lift = qbar * (CL + VPC.CLq * (VPC.c / (2 * Va_safe)) * state.q)
        F_drag = qbar * (CD + VPC.CDq * (VPC.c / (2 * Va_safe)) * state.q)

        # Transform Lift and Drag to Body Frame
        forces_moments.Fx = - (F_drag * math.cos(state.alpha)) + F_lift * math.sin(state.alpha)
        forces_moments.Fz = - (F_drag * math.sin(state.alpha)) - F_lift * math.cos(state.alpha)

        # Pitching Moment (Beard 4.5, first 3 terms)
        forces_moments.My = qbar * VPC.c * (VPC.CM0 + VPC.CMalpha * state.alpha + VPC.CMq * (VPC.c / (2 * Va_safe)) * state.q)

        # Lateral Aerodynamic Forces (Beard 4.14 first 4 terms)
        forces_moments.Fy = qbar * (VPC.CY0 + VPC.CYbeta * state.beta + VPC.CYp * (VPC.b / (2 * Va_safe)) * state.p + 
                                    VPC.CYr * (VPC.b / (2 * Va_safe)) * state.r)

        # Rolling Moment (Beard 4.15 first 4 terms)
        forces_moments.Mx = qbar * VPC.b * (VPC.Cl0 + VPC.Clbeta * state.beta + VPC.Clp * (VPC.b / (2 * Va_safe)) * state.p + 
                                            VPC.Clr * (VPC.b / (2 * Va_safe)) * state.r)

        # Yawing Moment (Beard 4.16 first 4 terms)
        forces_moments.Mz = qbar * VPC.b * (VPC.Cn0 + VPC.Cnbeta * state.beta + VPC.Cnp * (VPC.b / (2 * Va_safe)) * state.p + 
                                            VPC.Cnr * (VPC.b / (2 * Va_safe)) * state.r)

        # If small enough just set attributes to zero.
        eps_attr = 1e-10
        for attr in vars(forces_moments):
            value = getattr(forces_moments, attr)
            if abs(value) < eps_attr:
                setattr(forces_moments, attr, 0.0)

        return forces_moments
    
    def CalculatePropForces(self, Va, Throttle):
        """
        Computes forces and moments generated by the propeller.
        """
        rho = VPC.rho
        D = VPC.D_prop
        Vin = VPC.V_max * Throttle
        
        # Compute quadratic equation terms for rotation rate Omega
        a = (rho * D**5 * VPC.C_Q0) / (4 * math.pi**2)
        b = (rho * D**4 * Va * VPC.C_Q1) / (2 * math.pi) + (VPC.KQ * VPC.KQ) / VPC.R_motor # K_E = K_T = K_Q
        c = (rho * D**3 * Va**2 * VPC.C_Q2) - (VPC.KQ / VPC.R_motor) * Vin + (VPC.KQ * VPC.i0)
        
        
        # Check if the discriminant is negative
        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            Omega = 100  # Safe fallback value
        else:
            Omega = (-b + math.sqrt(discriminant)) / (2 * a)

        # Check if real: math.sqrt(b**2 - 4 * a * c) set it to 100
        # Omega = (-b + math.sqrt(b**2 - 4 * a * c)) / (2 * a)
        # J = (2 * math.pi * Va) / (Omega * D)
        if Omega == 0:
            Omega == 100
        
        J = (2 * math.pi * Va) / (Omega * D) 
        
        # Compute thrust and torque coefficients
        CT = VPC.C_T0 + VPC.C_T1 * J + VPC.C_T2 * J**2
        CQ = VPC.C_Q0 + VPC.C_Q1 * J + VPC.C_Q2 * J**2
        
        # Compute thrust and torque
        Fprop = (rho * Omega**2 * D**4 * CT) / (4 * math.pi**2)
        Mprop = (-rho * Omega**2 * D**5 * CQ) / (4 * math.pi**2)
        
        return Fprop, Mprop

    def controlForces(self, state, controls):
        """
        Computes aerodynamic forces and moments from control surfaces then adds the propeller forces.
        """
        qbar = 0.5 * VPC.rho * (state.Va ** 2) * VPC.S
        
        forces_moments = Inputs.forcesMoments()
        
        # Lift and Drag due to Elevator (Beard 4.6 & 4.7, 3rd term)
        F_lift_control = qbar * VPC.CLdeltaE * controls.Elevator
        F_drag_control = qbar * VPC.CDdeltaE * controls.Elevator
        
        # Transform Lift and Drag to Body Frame
        forces_moments.Fx = - (F_drag_control * math.cos(state.alpha)) + (F_lift_control * math.sin(state.alpha))
        forces_moments.Fz = - (F_drag_control * math.sin(state.alpha)) - (F_lift_control * math.cos(state.alpha))
        
        # Pitching Moment (Beard 4.5, 4th term)
        forces_moments.My = qbar * VPC.c * VPC.CMdeltaE * controls.Elevator
        
        # Lateral Forces and Moments (Beard 4.14, 4.15, 4.16, last 2 terms)
        forces_moments.Fy = qbar * (VPC.CYdeltaA * controls.Aileron + VPC.CYdeltaR * controls.Rudder)
        forces_moments.Mx = qbar * VPC.b * (VPC.CldeltaA * controls.Aileron + VPC.CldeltaR * controls.Rudder)
        forces_moments.Mz = qbar * VPC.b * (VPC.CndeltaA * controls.Aileron + VPC.CndeltaR * controls.Rudder)
        
        # Add Propeller Forces
        Fprop, Mprop = self.CalculatePropForces(state.Va, controls.Throttle)
        forces_moments.Fx += Fprop
        forces_moments.Mx += Mprop
        
        return forces_moments
    
    def CalculateAirspeed(self, state, wind):
        """
        Computes airspeed parameters (Va, alpha, beta) considering wind effects.
        """
        
        # Step 0: Account for wind = None
        if wind is None:
            wind = self.windModel.windState
            # print(f"Wind: {wind}")
            
        # Step 1: Compute Wind Azimuth and Elevation 
        Ws = max(1e-6, math.sqrt(wind.Wn**2 + wind.We**2 + wind.Wd**2))  # Prevent divide by zero
        chi_w = math.atan2(wind.We, wind.Wn)  # Wind azimuth angle 
        gamma_w = -math.asin(wind.Wd / Ws)  # Wind elevation angle 

        # Step 2: Construct Azimuth-Elevation Rotation Matrix 
        R_inertial_to_wind = [
            [math.cos(chi_w) * math.cos(gamma_w), math.sin(chi_w) * math.cos(gamma_w), -math.sin(gamma_w)],
            [-math.sin(chi_w), math.cos(chi_w), 0],
            [math.cos(chi_w) * math.sin(gamma_w), math.sin(chi_w) * math.sin(gamma_w), math.cos(gamma_w)]
        ]

        # Step 3: Transpose for Wind Frame to Inertial Frame Transformation 
        R_wind_to_inertial = MatrixMath.transpose(R_inertial_to_wind)  

        # Step 4: Rotate Wind Gust from Wind Frame to Inertial Frame
        gust_wind = [[wind.Wu], [wind.Wv], [wind.Ww]]  # Gust vector in wind frame
        gust_wind_inertial = MatrixMath.multiply(R_wind_to_inertial, gust_wind)  # Rotate to inertial frame

        # Step 5: Compute Total Wind in Inertial Frame
        W_inertial = [
            [wind.Wn + gust_wind_inertial[0][0]],
            [wind.We + gust_wind_inertial[1][0]],
            [wind.Wd + gust_wind_inertial[2][0]]
        ]

        # Step 6: Rotate Wind from Inertial → Body Frame using Euler DCM 
        R_inertial_to_body = Rotations.euler2DCM(state.yaw, state.pitch, state.roll)
        W_body = MatrixMath.multiply(R_inertial_to_body, W_inertial)  

        # Step 7: Compute Airspeed Vector 
        u_b = state.u - W_body[0][0]
        v_b = state.v - W_body[1][0]
        w_b = state.w - W_body[2][0]

        # Step 8: Compute Airspeed Magnitude 
        Va = math.sqrt(u_b**2 + v_b**2 + w_b**2) 

        # Step 9: Compute Angle of Attack and Sideslip
        alpha = math.atan2(w_b, u_b) if Va > 0 else 0.0
        beta = math.asin((v_b / Va)) if Va > 0 else 0.0

        return Va, alpha, beta 

    def updateForces(self, state, controls, wind=None):
        """
        Computes and updates all forces acting on the vehicle.
        """
        
        # Update Va, alpha, and beta using wind
        state.Va, state.alpha, state.beta = self.CalculateAirspeed(state, wind)

        # Compute forces
        gravity_forces = self.gravityForces(state)
        aero_forces = self.aeroForces(state)
        control_forces = self.controlForces(state, controls)
        
        # Sum forces
        total_forces = Inputs.forcesMoments(
            Fx=gravity_forces.Fx + aero_forces.Fx + control_forces.Fx,
            Fy=gravity_forces.Fy + aero_forces.Fy + control_forces.Fy,
            Fz=gravity_forces.Fz + aero_forces.Fz + control_forces.Fz,
            Mx=gravity_forces.Mx + aero_forces.Mx + control_forces.Mx,
            My=gravity_forces.My + aero_forces.My + control_forces.My,
            Mz=gravity_forces.Mz + aero_forces.Mz + control_forces.Mz
        )
        
        return total_forces
