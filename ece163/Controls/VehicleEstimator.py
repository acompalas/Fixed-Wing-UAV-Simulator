"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains a classes for estimator complementary filters. Methods were sourced from ECE163 Estimators handout.
"""

import math
from ..Containers import Controls
from ..Containers import Sensors
from ..Containers import States
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleDynamicsModel as VDM
from ..Sensors import SensorsModel
from ..Utilities import MatrixMath as MM
from ..Utilities import Rotations

class LowPassFilter:
    """
    Implements a first-order low-pass filter for smoothing noisy sensor data.
    """
    def __init__(self, dT=VPC.dT, cutoff=1):
        """
        Initializes the LowPassFilter.
        """
        self.dT = dT
        self.cutoff = cutoff
        self.a = 2 * math.pi * cutoff
        self.expTerm = math.exp(-self.a * self.dT) 
        self.output = 0  

    def reset(self):
        """
        Resets the internal state of the filter.
        """
        self.output = 0

    def update(self, input):
        """
        Updates the low-pass filter based on input and previous output.
        """
        self.output = self.expTerm * self.output + (1 - self.expTerm) * input
        return self.output


class VehicleEstimator:
    """
    Implements state estimation for UAV attitude, altitude, course, and airspeed using complementary filters.
    """
    def __init__(self, dT=VPC.dT, gains=Controls.VehicleEstimatorGains(), sensorsModel=SensorsModel.SensorsModel()):
        """
        Initializes the VehicleEstimator.
        
        Parameters:
        dT – Update time step.
        gains – The gains for the estimation filters.
        sensorsModel – The SensorsModel class object. Contains the vehicle state.
        """
        self.dT = dT
        self.gains = gains
        self.sensorsModel = sensorsModel
        
        # Initialize state values to VPC initial conditions
        self.estimatedState = States.vehicleState()
        self.estimatedState.pd = VPC.InitialDownPosition
        self.estimatedState.Va = VPC.InitialSpeed
        
        self.lowPassFilter = LowPassFilter(dT, gains.lowPassCutoff_h)
        self.biases = {
            "gyro": [[0], [0], [0]],
            "pitot": 0,
            "chi": 0,
            "ascent_rate": 0,
            "altitude_gps": 0
        }

    def reset(self):
        """
        Resets the state estimate, biases, and low-pass filter.
        """
        self.estimatedState = States.vehicleState()
        self.estimatedState.pd = VPC.InitialDownPosition
        self.estimatedState.Va = VPC.InitialSpeed
        self.lowPassFilter.reset()

        self.biases = {
            "gyro": [[0.0], [0.0], [0.0]],
            "pitot": 0.0,
            "chi": 0.0,
            "ascent_rate": 0.0,
            "altitude_gps": 0.0
        }


    def getEstimatedState(self):
        """
        Returns the estimated vehicle state.
        """
        return self.estimatedState

    def getEstimatorGains(self):
        """
        Returns the estimator gains.
        """
        return self.gains

    def setEstimatorGains(self, gains=Controls.VehicleEstimatorGains()):
        """
        Sets the estimator gains.
        """
        self.gains = gains

    def setEstimatedState(self, estimatedState=States.vehicleState()):
        """
        Sets the estimated vehicle state.
        """
        self.estimatedState = estimatedState

    def setEstimatorBiases(self, estimatedGyroBias=[[0], [0], [0]], estimatedPitotBias=0, 
                            estimatedChiBias=0, estimatedAscentRate=0, estimatedAltitudeGPSBias=0):
        """
        Sets the estimator biases.
        
        Parameters:
        estimatedGyroBias – The estimator gyro biases for all 3 axes.
        estimatedPitotBias – The estimated pitot bias.
        estimatedChiBias – The estimated course bias.
        estimatedAscentRate – The estimated ascent rate (alt_dot).
        estimatedAltitudeGPSBias – The estimated altitude bias from GPS.
        """
        self.biases["gyro"] = estimatedGyroBias
        self.biases["pitot"] = estimatedPitotBias
        self.biases["chi"] = estimatedChiBias
        self.biases["ascent_rate"] = estimatedAscentRate
        self.biases["altitude_gps"] = estimatedAltitudeGPSBias


    def estimateAttitude(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState()):
            """
            Estimates the vehicle's attitude (DCM) and angular rates (p, q, r) using a complementary filter.
            """
                
            if "gyro" not in self.biases:
                self.biases["gyro"] = [[0.0], [0.0], [0.0]]
                
            # Unpack Sensor Measurements
            p_meas, q_meas, r_meas = sensorData.gyro_x, sensorData.gyro_y, sensorData.gyro_z
            ax, ay, az = sensorData.accel_x, sensorData.accel_y, sensorData.accel_z
            mx, my, mz = sensorData.mag_x, sensorData.mag_y, sensorData.mag_z
            
            # Normalize the known inertial gravity vector
            g_inertial = MM.vectorNorm([[0], [0], [VPC.g0]])  # Normalized gravity vector
            
            # Normalize the known inertial magnetic field from the IGRF model
            h_inertial = MM.vectorNorm(VSC.magfield)  # Unit vector in inertial frame
            
            # Normalize Acceleration and Magnetometer Readings
            a_body = MM.vectorNorm([[ax], [ay], [az]])
            h_body = MM.vectorNorm([[mx], [my], [mz]])
            
            # Store the boolean for accelerometer usage  
            accel_mag = math.hypot(ax, ay, az)    
            use_accel = (0.9 * VPC.g0 <= accel_mag <= 1.1 * VPC.g0) 
            
            # Initialize Attitude Errors to Zero Vector
            omega_mag = [[0.0], [0.0], [0.0]]
            omega_acc = [[0.0], [0.0], [0.0]]
            
            # Compute Magnetometer Error
            omega_mag = MM.crossProduct(h_body, MM.multiply(estimatedState.R, h_inertial))
            
            # Update bias with magnetometer error
            bias_dot = MM.scalarMultiply(-self.gains.Ki_mag, omega_mag)
            
            # If accelerometer is valid, compute Accelerometer Error
            if use_accel:
                omega_acc = MM.crossProduct(a_body, MM.multiply(estimatedState.R, g_inertial))
                bias_dot = MM.subtract(bias_dot, MM.scalarMultiply(self.gains.Ki_acc, omega_acc))

            # Integrate bias
            b_plus = MM.add(self.biases["gyro"], MM.scalarMultiply(self.dT, bias_dot))
            gyro_unbiased = MM.subtract([[p_meas], [q_meas], [r_meas]], b_plus)
            
            # Update omega x
            omega_corrected = MM.add(gyro_unbiased, MM.scalarMultiply(self.gains.Kp_mag, omega_mag))
            if use_accel:
                omega_corrected = MM.add(omega_corrected, MM.scalarMultiply(self.gains.Kp_acc, omega_acc))
                
            # Set Temp States
            temp_state = States.vehicleState()
            state_dot = States.vehicleState()
            temp_state.p, temp_state.q, temp_state.r = omega_corrected[0][0], omega_corrected[1][0], omega_corrected[2][0]
            
            # Update R
            vdm = VDM.VehicleDynamicsModel()
            exp_matrix = vdm.Rexp(self.dT, temp_state, state_dot)
            R_new = MM.multiply(exp_matrix, estimatedState.R)

            # Return new biases, omega, and R
            return b_plus, gyro_unbiased, R_new

    def estimateAirspeed(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState()):
        """
        Estimates the vehicle's airspeed (Va) by combining the estimated attitude (gyroscope and accelerometer) 
        and GPS using a complementary filter.

        Uses the following gains from the class gain attribute: Kp_Va, Ki_Va. 
        Larger values for these gains means you trust the pitot more.
        """
        
        # Initialize airspeed estimates
        Va_est = estimatedState.Va if estimatedState.Va is not None else 0.0
        Va_pitot = math.sqrt(2 * abs(sensorData.pitot) / VPC.rho) if sensorData.pitot is not None else 0.0
        
        # Initialize bias
        b_Va = self.biases["pitot"] if "pitot" in self.biases else 0.0
          
        # Unpack body acceleration from sensors
        a_body = [[sensorData.accel_x], [sensorData.accel_y], [sensorData.accel_z]]
        
        # Compute gravity in body frame using DCM
        g_inertial = [[0], [0], [VPC.g0]]
        g_body = MM.multiply(estimatedState.R, g_inertial)
        
        # Compute corrected body x-acceleration
        ax_body = a_body[0][0] + g_body[0][0]
        
        # Bias correction
        b_Va_dot = -self.gains.Ki_Va * (Va_pitot - Va_est)
        b_Va += b_Va_dot * self.dT  # Integrate bias estimate

        # Apply airspeed complementary filter
        Va_dot = (ax_body - b_Va) + self.gains.Kp_Va * (Va_pitot - Va_est)  # Compute rate of change of airspeed
        Va_est += Va_dot * self.dT  # Integrate to get estimated airspeed

        return b_Va, Va_est
        
    def estimateAltitude(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState()):
        """
        Estimates the vehicle's altitude (h) by passing barometer data through a high-pass filter 
        and GPS data through a low-pass filter and combining them using a complementary filter.

        Uses the following gains from the class gain attribute: Kp_h, Ki_h. 
        Larger values for these gains means you trust the baro more
        """
        # Initialize h estimate
        h_est = -estimatedState.pd
            
        # Initialize h_dot estimate
        h_dot_est = self.biases["ascent_rate"] if "ascent_rate" in self.biases else 0.0
            
        # Initialize previous bias
        b_GPS = self.biases["altitude_gps"] if "altitude_gps" in self.biases else 0.0
        
        # Extract barometric altitude and apply low-pass filter
        h_baro = sensorData.baro
        lpf = self.lowPassFilter
        lpf.reset() # Ensure that output is cleared before using it
        h_lpf = lpf.update(h_baro)
        
        # Compute inertial acceleration in the up (z) direction
        R_T = MM.transpose(estimatedState.R)  # Transpose of DCM
        accel_body = [[sensorData.accel_x], [sensorData.accel_y], [sensorData.accel_z]]
        accel_inertial = MM.multiply(R_T, accel_body)  
        ia_up = accel_inertial[2][0] + VPC.g0  # Extract vertical acceleration
        
        # Estimate ascent rate (vertical speed)
        h_dot_est = h_dot_est + (ia_up + self.gains.Ki_h * (h_lpf - h_est)) * self.dT
        
        # Estimate altitude using complementary filter
        h_est = h_est + (self.gains.Kp_h * (h_lpf - h_est) + h_dot_est) * self.dT

        # GPS update
        if self.sensorsModel.updateTicks % self.sensorsModel.gpsTickUpdate == 1:
            gps_alt = sensorData.gps_alt
            dT_GPS = self.dT
            
            # Compute Bias Correction
            b_GPS_dot = -self.gains.Ki_h_gps * (gps_alt - h_est)  # Compute bias correction rate
            b_GPS = b_GPS + b_GPS_dot * dT_GPS  # Integrate GPS bias
            
            # Apply altitude correction
            h_est = h_est + (self.gains.Kp_h_gps * (gps_alt - h_est) - b_GPS) * dT_GPS
        else:
            h_est = h_est - b_GPS  # Remove GPS bias correction when no update

        return h_est, h_dot_est, b_GPS

    def estimateCourse(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState()):
        """
        Function to estimate the vehicle's course (chi) using GPS. For the internal course rate, we are using dot{psi} rather than dot{chi} 
        and letting the bias estimate clean it up.

        Uses the following gains from the class gain attribute: Kp_chi, Ki_chi. Larger values for these gains means you trust the GPS more.
        """
        
        # Initialize course estimate
        chi_est = sensorData.gps_cog if sensorData.gps_cog is not None else estimatedState.yaw if estimatedState.yaw is not None else 0.0
        
        # Initialize course bias
        b_chi = self.biases["chi"] if "chi" in self.biases else 0.0
        
        # Estimate Course Rate 
        phi = estimatedState.roll
        theta = estimatedState.pitch
        q = estimatedState.q
        r = estimatedState.r

        # chi_dot_est = (1 / math.cos(theta)) * (q * math.sin(phi) + r * math.cos(phi))
        chi_dot_est = estimatedState.r
        
        # Apply low-pass filter to smooth chi_dot_est
        # lpf = self.lowPassFilter
        # lpf.reset()  # Ensure filter is cleared before using it
        # chi_dot_est = lpf.update(chi_dot_est)
        
        # GPS Update
        if self.sensorsModel.updateTicks % self.sensorsModel.gpsTickUpdate == 1:
            gps_cog = sensorData.gps_cog
            dT_GPS = self.dT

            chi_error = gps_cog - chi_est
            # chi_error = (chi_error + math.pi) % (2 * math.pi) - math.pi
            chi_error = math.atan2(math.sin(chi_error), math.cos(chi_error))
            b_chi_dot = -self.gains.Ki_chi * chi_error
            b_chi += b_chi_dot * dT_GPS
            chi_est += (self.gains.Kp_chi * chi_error - b_chi) * dT_GPS
        else:
            chi_est += (chi_dot_est - b_chi) * self.dT
            
        # chi_est = (chi_est + math.pi) % (2 * math.pi) - math.pi
        chi_est = math.atan2(math.sin(chi_est), math.cos(chi_est))
        
        return b_chi, chi_est

    def Update(self):
        """
        Updates the vehicle state estimation using complementary filters.

        Update Order:
        1. Attitude
        2. Altitude
        3. Airspeed
        4. Course

        Uses noisy sensor data from `sensorsModel` and updates the estimated state.
        """

        # Get the latest sensor readings
        sensorData = self.sensorsModel.sensorsNoisy

        # Get the current estimated state
        estimatedState = self.getEstimatedState()

        # Step 1: Update Attitude (DCM, Angular Rates)
        gyro_bias, gyro_unbiased, R_new = self.estimateAttitude(sensorData, estimatedState)
        
        # Assemble new state gyro rates
        estimatedState.p = gyro_unbiased[0][0]
        estimatedState.q = gyro_unbiased[1][0]
        estimatedState.r = gyro_unbiased[2][0]
        
        # Set R to use in other functions
        estimatedState.R = R_new
        
        # Set gyro bias
        self.biases["gyro"] = gyro_bias

        # Step 2: Update Altitude (h, h_dot, and altitude bias)
        h_est, h_dot_est, b_h = self.estimateAltitude(sensorData, estimatedState)
        estimatedState.pd = -h_est
        self.biases["ascent_rate"] = h_dot_est
        self.biases["altitude_gps"] = b_h

        # Step 3: Update Airspeed (Va and pitot bias)
        pitot_bias, Va_new = self.estimateAirspeed(sensorData, estimatedState)
        estimatedState.Va = Va_new
        self.biases["pitot"] = pitot_bias

        # Step 4: Update Course (Chi and course bias)
        chi_bias, chi_new = self.estimateCourse(sensorData, estimatedState)
        estimatedState.chi = chi_new
        self.biases["chi"] = chi_bias
        
        # Calculating yaw, pitch, and roll
        estimatedState.yaw, estimatedState.pitch, estimatedState.roll = Rotations.dcm2Euler(estimatedState.R)
        
        # Correctly update the internal state
        self.setEstimatedState(estimatedState)

 