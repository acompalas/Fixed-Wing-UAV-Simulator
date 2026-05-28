"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains a class for vehicle kinematics and dynamics. Formulas sourced from the 12 State Model in Beard textbook
"""

import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleDynamicsModel:
    def __init__(self):
        """
        Initializes the vehicle dynamics model by setting up the state, derivative of the state,
        and simulation time step.
        """

        self.state = States.vehicleState()
        self.state_dot = States.vehicleState()
        self.delta_t = VPC.dT

    def reset(self):
        """
        Resets the vehicle state and its derivative to default values.
        """
        self.state = States.vehicleState()
        self.state_dot = States.vehicleState()
        self.state.R = [[1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]]

    def getVehicleState(self):
        """
        Returns the current vehicle state.
        """
        return self.state

    def setVehicleState(self, new_state):
        """
        Sets the current vehicle state.
        """
        self.state = new_state

    def getVehicleDerivative(self):
        """
        Returns the current derivative of the vehicle state.
        """
        return self.state_dot

    def setVehicleDerivative(self, new_state_dot):
        """
        Sets the derivative of the vehicle state.
        """
        self.state_dot = new_state_dot
        
    def Update(self, forcesMoments):
        """
        Updates the vehicle's state for the current timestep.
        """
        state = self.getVehicleState()
        self.dot = self.derivative(state, forcesMoments)
        self.state = self.IntegrateState(self.delta_t, state, self.dot)


    def derivative(self, state, forces_moments):
        """
        Calculates the derivative of the state given the current state and input forces/moments.
        """
        dot = States.vehicleState()  # Initialize the derivative object

        # 1. Position derivatives (NED frame from body velocities)
        velocity_body = [[state.u], [state.v], [state.w]]  # Body-frame velocities
        velocity_ned = MatrixMath.multiply(MatrixMath.transpose(state.R), velocity_body)  # R^T * velocity_body
        dot.pn, dot.pe, dot.pd = velocity_ned[0][0], velocity_ned[1][0], velocity_ned[2][0]

        # 2. Velocity derivatives (body frame)
        m = VPC.mass
        Fx, Fy, Fz = forces_moments.Fx, forces_moments.Fy, forces_moments.Fz
        omega_cross = MatrixMath.skew(state.p, state.q, state.r)  # Skew matrix of angular rates
        omega_cross_vel = MatrixMath.multiply(omega_cross, velocity_body)  # Angular velocity cross body velocity
        dot.u = Fx / m - omega_cross_vel[0][0]
        dot.v = Fy / m - omega_cross_vel[1][0]
        dot.w = Fz / m - omega_cross_vel[2][0]

        # 3. Angular velocity derivatives (Euler angles)
        phi, theta = state.roll, state.pitch
        p, q, r = state.p, state.q, state.r
        tan_theta = math.tan(theta)
        sec_theta = 1 / math.cos(theta)

        G = [
            [1, math.sin(phi) * tan_theta, math.cos(phi) * tan_theta],
            [0, math.cos(phi), -math.sin(phi)],
            [0, math.sin(phi) * sec_theta, math.cos(phi) * sec_theta]
        ]
        angular_rates = [[p], [q], [r]]
        euler_rates = MatrixMath.multiply(G, angular_rates)
        dot.roll, dot.pitch, dot.yaw = euler_rates[0][0], euler_rates[1][0], euler_rates[2][0]

        # 4. Angular accelerations (body frame)
        J = VPC.Jbody  # Inertia matrix
        J_inv = VPC.JinvBody  # Inverse of inertia matrix
        torques = [[forces_moments.Mx], [forces_moments.My], [forces_moments.Mz]]
        omega = [[state.p], [state.q], [state.r]]  # Angular velocities
        omega_cross_J_omega = MatrixMath.multiply(MatrixMath.skew(*[row[0] for row in omega]), MatrixMath.multiply(J, omega))
        angular_accel = MatrixMath.subtract(MatrixMath.multiply(J_inv, torques), MatrixMath.multiply(J_inv, omega_cross_J_omega))
        dot.p, dot.q, dot.r = angular_accel[0][0], angular_accel[1][0], angular_accel[2][0]
        
        # 5. DCM derivative
        omega_cross = MatrixMath.skew(state.p, state.q, state.r)  # Skew matrix of angular rates
        dot.R = MatrixMath.multiply(MatrixMath.scalarMultiply(-1, omega_cross), state.R)  # -[\omega_\times] R

        return dot
    
    def Rexp(self, delta_t, state, state_dot):
        """
        Calculates the matrix exponential for propagating the rotation matrix.
        """
        # Compute midpoint angular velocity
        p_mid = state.p + 0.5 * state_dot.p * delta_t
        q_mid = state.q + 0.5 * state_dot.q * delta_t
        r_mid = state.r + 0.5 * state_dot.r * delta_t

        # Compute angular velocity magnitude ||ω||
        omega_norm = math.hypot(p_mid, q_mid, r_mid)

        # Skew-symmetric matrix [ω×]
        omega_skew = MatrixMath.skew(p_mid, q_mid, r_mid)

        # Compute sin and cos terms based on ω * Δt
        if omega_norm > 0.2:  # Regular calculation for larger values
            sin_term = math.sin(omega_norm * delta_t) / omega_norm
            cos_term = (1 - math.cos(omega_norm * delta_t)) / (omega_norm**2)
        else:  # Use Taylor series approximation for small values
            sin_term = delta_t - (delta_t**3 * omega_norm**2) / 6 + (delta_t**5 * omega_norm**4) / 120
            cos_term = (delta_t**2) / 2 - (delta_t**4 * omega_norm**2) / 24 + (delta_t**6 * omega_norm**4) / 720
            
        def identity(size):
            """
            Creates an identity matrix of the specified size.
            """
            return [[1 if i == j else 0 for j in range(size)] for i in range(size)]

        # Calculate the matrix exponential
        I = identity(3)
        omega_skew_squared = MatrixMath.multiply(omega_skew, omega_skew)
        exp_matrix = MatrixMath.add(
            MatrixMath.subtract(I, MatrixMath.scalarMultiply(sin_term, omega_skew)),
            MatrixMath.scalarMultiply(cos_term, omega_skew_squared),
        )

        return exp_matrix
    
    def ForwardEuler(self, dT, state, dot):
        """
        Performs the Forward Euler method to compute the next state.
        """
        # Create a new state object to store the results
        new_state = States.vehicleState()

        # Update position (NED frame)
        new_state.pn = state.pn + dot.pn * dT
        new_state.pe = state.pe + dot.pe * dT
        new_state.pd = state.pd + dot.pd * dT

        # Update velocity (body frame)
        new_state.u = state.u + dot.u * dT
        new_state.v = state.v + dot.v * dT
        new_state.w = state.w + dot.w * dT

        # Update angular velocity (body frame)
        new_state.p = state.p + dot.p * dT
        new_state.q = state.q + dot.q * dT
        new_state.r = state.r + dot.r * dT

        # Update Euler angles
        new_state.roll = state.roll + dot.roll * dT
        new_state.pitch = state.pitch + dot.pitch * dT
        new_state.yaw = state.yaw + dot.yaw * dT

        # Update the Direction Cosine Matrix (DCM)
        new_state.R = MatrixMath.add(state.R, MatrixMath.scalarMultiply(dT, dot.R))

        # Copy alpha, beta, Va, and chi from the current state (not updated here)
        new_state.alpha = state.alpha
        new_state.beta = state.beta
        new_state.Va = state.Va
        new_state.chi = state.chi

        return new_state


    def IntegrateState(self, dT, state, dot):
        """
        Integrates the state using Forward Euler for most variables and matrix exponential for R.
        """
        # Use Forward Euler for state variables
        newState = self.ForwardEuler(dT, state, dot)

        # Update the attitude (R) using matrix exponential
        R_exp = self.Rexp(dT, state, dot)
        newState.R = MatrixMath.multiply(R_exp, state.R)

        # Derive Euler angles (roll, pitch, yaw) from the updated rotation matrix
        newState.yaw, newState.pitch, newState.roll = Rotations.dcm2Euler(newState.R)
        
        if math.isclose(newState.yaw, math.pi / 2, abs_tol=1e-6):
            newState.yaw = 0  # Correct yaw alignment error

        # Copy alpha, beta, and Va (held constant)
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.Va = state.Va

        # Calculate the course angle chi
        newState.chi = math.atan2(dot.pe, dot.pn) 
        return newState



