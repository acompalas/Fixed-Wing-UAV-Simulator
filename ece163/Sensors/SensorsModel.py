"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains a classes for sensor behavious. Formulas sourced from the Chapter 7 in the Beard textbook and the Gauss-Markov Handout.
"""

import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel

class GaussMarkov():
    def __init__(self, dT=VPC.dT, tau=1e6, eta=0.0):
        """
        Initializes the GaussMarkov process.
        
        Parameters:
            dT  : Time step [s]
            tau : Correlation time [s]
            eta : White noise process 
        """
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0.0

    def reset(self):
        """
        Resets the internal state of the Gauss-Markov process.
        """
        self.v = 0.0

    def update(self, vnoise=None):
        """
        Advances the Gauss-Markov process by one time-step.
        """
        # Calculate the decay factor and the noise scaling factor
        a = math.exp(-self.dT / self.tau)
        
        # Use external noise if provided; otherwise, sample from standard normal
        if vnoise is None:
            vnoise = random.gauss(0,self.eta) 
            
        # Update the internal state using the discrete-time Gauss-Markov update
        self.v = a * self.v + vnoise
        
        return self.v 

class GaussMarkovXYZ():
    def __init__(self, dT=VPC.dT, tauX=1e6, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None):
        """
        Function to aggregate three Gauss-Markov models into a triplet that returns the X, Y, and Z axes of the time-varying drift
        """
        # If tau/eta for Y or Z are not provided, use X defaults
        self.tauX = tauX
        self.etaX = etaX
        self.tauY = tauY if tauY is not None else tauX
        self.etaY = etaY if etaY is not None else etaX
        self.tauZ = tauZ if tauZ is not None else tauX
        self.etaZ = etaZ if etaZ is not None else etaX

        # Create three independent Gauss-Markov processes
        self.gx = GaussMarkov(dT, self.tauX, self.etaX)
        self.gy = GaussMarkov(dT, self.tauY, self.etaY)
        self.gz = GaussMarkov(dT, self.tauZ, self.etaZ)

    def reset(self):
        """
        Wrapper function that resets the GaussMarkovXYZ models
        """
        self.gx.reset()
        self.gy.reset()
        self.gz.reset()

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):
        """
        Function that updates the Gauss-Markov processes, and returns the updated values 
        (as well as updating the internal values that holds until the next call of the function.
        """
        vX = self.gx.update(vXnoise)
        vY = self.gy.update(vYnoise)
        vZ = self.gz.update(vZnoise)
        return vX, vY, vZ


class SensorsModel():
    def __init__(self, 
                 aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(), 
                 taugyro=VSC.gyro_tau, 
                 etagyro=VSC.gyro_eta, 
                 tauGPS=VSC.GPS_tau, 
                 etaGPSHorizontal=VSC.GPS_etaHorizontal, 
                 etaGPSVertical=VSC.GPS_etaVertical, 
                 gpsUpdateHz=VSC.GPS_rate):
        """
        Function to gather all of the white noise standard deviations into a single vehicleSensor class object. 
        These will be used as the input to generating the white noise added to each sensor when generating the noisy sensor data.
        """
        # Store the reference to the vehicle aerodynamics model.
        self.aeroModel = aeroModel
        self.dT = self.aeroModel.vehicleDynamics.delta_t

        # Create containers for sensor data.
        self.sensorsTrue = Sensors.vehicleSensors()         # sensorsTrue will hold the ideal (noise-free) sensor outputs.
        self.sensorsBiases = Sensors.vehicleSensors()       # sensorsBiases holds the static biases.
        self.sensorSigmas  = Sensors.vehicleSensors()       # sensorSigmas holds the noise standard deviations for each sensor.
        self.sensorsNoisy  = Sensors.vehicleSensors()       # sensorsNoisy holds the actual noisy sensor outputs.

        # Initialize the biases and sigmas (using methods defined later).
        self.initializeBiases()
        self.initializeSigmas()

        # Initialize the Gauss-Markov process for the gyro drifting bias.
        # This process will be used to add a slowly varying bias to the gyro measurements.
        self.gyroGM = GaussMarkov(self.dT, taugyro, etagyro)

        # Initialize Gauss-Markov processes for the GPS drifting bias.
        # Here, we use one for the horizontal (north/east) components and one for the vertical (altitude) component.
        self.gpsGM_horizontal = GaussMarkov(self.dT, tauGPS, etaGPSHorizontal)
        self.gpsGM_vertical   = GaussMarkov(self.dT, tauGPS, etaGPSVertical)

        # Initialize the GPS timing system:
        # updateTicks is a counter to keep track of simulation ticks.
        self.updateTicks = 0                                
        # gpsTickUpdate is the number of dT intervals that make up one GPS update period.
        self.gpsTickUpdate = int(round(1.0 / (gpsUpdateHz * self.dT)))

    def initializeBiases(self, 
                         gyroBias=VSC.gyro_bias, 
                         accelBias=VSC.accel_bias, 
                         magBias=VSC.mag_bias, 
                         baroBias=VSC.baro_bias, 
                         pitotBias=VSC.pitot_bias):
        """
        Function to generate the biases for each of the sensors. Biases are set with a uniform 
        random number from -1 to 1 that is then multiplied by the bias scaling factor. The biases 
        for all sensors are returned as a Sensors.vehicleSensors class object. Note that GPS is 
        an unbiased sensor (though noisy), thus all the GPS biases are set to 0.0.
        """
        biases = Sensors.vehicleSensors()
        # Set gyro biases (X, Y, Z)
        biases.gyro_x = random.uniform(-1, 1) * gyroBias
        biases.gyro_y = random.uniform(-1, 1) * gyroBias
        biases.gyro_z = random.uniform(-1, 1) * gyroBias
        
        # Set accelerometer biases (X, Y, Z)
        biases.accel_x = random.uniform(-1, 1) * accelBias
        biases.accel_y = random.uniform(-1, 1) * accelBias
        biases.accel_z = random.uniform(-1, 1) * accelBias
        
        # Set magnetometer biases (X, Y, Z)
        biases.mag_x = random.uniform(-1, 1) * magBias
        biases.mag_y = random.uniform(-1, 1) * magBias
        biases.mag_z = random.uniform(-1, 1) * magBias
        
        # Set pressure sensor biases for barometer and pitot
        biases.baro = random.uniform(-1, 1) * baroBias
        biases.pitot = random.uniform(-1, 1) * pitotBias
        
        # GPS sensors are assumed unbiased
        biases.gps_n = 0.0
        biases.gps_e = 0.0
        biases.gps_alt = 0.0
        biases.gps_sog = 0.0
        biases.gps_cog = 0.0
        
        # Save the generated biases in the sensorsBiases container
        self.sensorsBiases = biases

    def initializeSigmas(self, 
                         gyroSigma=VSC.gyro_sigma, 
                         accelSigma=VSC.accel_sigma, 
                         magSigma=VSC.mag_sigma, 
                         baroSigma=VSC.baro_sigma, 
                         pitotSigma=VSC.pitot_sigma, 
                         gpsSigmaHorizontal=VSC.GPS_sigmaHorizontal, 
                         gpsSigmaVertical=VSC.GPS_sigmaVertical, 
                         gpsSigmaSOG=VSC.GPS_sigmaSOG, 
                         gpsSigmaCOG=VSC.GPS_sigmaCOG):
        """
        Function to gather all of the white noise standard deviations into a single vehicleSensor class object. 
        These values are used as the input for generating the white noise added to each sensor when creating 
        the noisy sensor data.
        """
        sigmas = Sensors.vehicleSensors()
        # Set gyro white noise values (X, Y, Z)
        sigmas.gyro_x = gyroSigma
        sigmas.gyro_y = gyroSigma
        sigmas.gyro_z = gyroSigma
        
        # Set accelerometer white noise values (X, Y, Z)
        sigmas.accel_x = accelSigma
        sigmas.accel_y = accelSigma
        sigmas.accel_z = accelSigma
        
        # Set magnetometer white noise values (X, Y, Z)
        sigmas.mag_x = magSigma
        sigmas.mag_y = magSigma
        sigmas.mag_z = magSigma
        
        # Set white noise values for pressure sensors
        sigmas.baro = baroSigma
        sigmas.pitot = pitotSigma
        
        # Set white noise values for GPS sensor outputs
        sigmas.gps_n = gpsSigmaHorizontal
        sigmas.gps_e = gpsSigmaHorizontal
        sigmas.gps_alt = gpsSigmaVertical
        sigmas.gps_sog = gpsSigmaSOG
        sigmas.gps_cog = gpsSigmaCOG
        
        # Save the gathered sigmas in the sensorSigmas container
        self.sensorSigmas = sigmas

    def updateGPSTrue(self, state, dot):
        """
        Function to update the GPS sensor state. Uses Beard 7.18, 7.19, 7.20, 7.21, 7.22
        """
        # GPS position (NED)
        gps_n = state.pn
        gps_e = state.pe
        gps_alt = -state.pd  # Altitude (positive up)

        # Extract airspeed and yaw
        Va = state.Va
        psi = state.yaw

        # Compute wind components (solving for w_n, w_e)
        w_n = dot.pn - Va * math.cos(psi)
        w_e = dot.pe - Va * math.sin(psi)

        # Compute GPS velocity using Beard's formulas
        Vn = Va * math.cos(psi) + w_n
        Ve = Va * math.sin(psi) + w_e
        
        # Compute Speed Over Ground (SOG)
        gps_sog = math.hypot(Vn, Ve)

        # Compute Course Over Ground (COG)
        gps_cog = math.atan2(Ve, Vn)
        
        return gps_n, gps_e, gps_alt, gps_sog, gps_cog

    def updateAccelsTrue(self, state, dot):
        """
        Function to update the accelerometer sensor.
        Returns the body-frame specific force (ideal accelerometer measurements) in m/s^2.
        """
        # Beard 7.1
        g = VPC.g0
        accel_x = dot.u + state.q * state.w - state.r * state.v + g * math.sin(state.pitch)
        accel_y = dot.v + state.r * state.u - state.p * state.w - g * math.cos(state.pitch) * math.sin(state.roll)
        accel_z = dot.w + state.p * state.v - state.q * state.u - g * math.cos(state.pitch) * math.cos(state.roll)
        return accel_x, accel_y, accel_z

    def updateMagsTrue(self, state):
        """
        Function to update the magnetometer sensor.
        Returns the ideal (noise-free) magnetometer measurements in the body frame in nT.
        """
        # Define Earth's magnetic field in NED as a column vector
        mag_field_NED = VSC.magfield
        
        # Transform the magnetic field into the body frame using the vehicle's DCM.
        mag_body = MatrixMath.multiply(state.R, mag_field_NED)      # Beard 7.12
        
        # Extract the components from the resulting column vector.
        mag_x = mag_body[0][0]
        mag_y = mag_body[1][0]
        mag_z = mag_body[2][0]
        
        return mag_x, mag_y, mag_z


    def updateGyrosTrue(self, state):
        """
        Function to update the rate gyro sensor.
        This function returns the ideal (noise-free) gyro measurements,
        which are assumed to be the body rates: p, q, and r.
        """
        # Beard 7.5
        return state.p, state.q, state.r

    def updatePressureSensorsTrue(self, state):
        """
        Function to update the pressure sensors onboard the aircraft.
        """
        # Barometric pressure using static pressure equation (Beard 7.7)
        baro = VSC.Pground - (VPC.rho * VPC.g0 * (-state.pd))
        
        # Pitot pressure using dynamic pressure equation (Beard 7.10)
        pitot = 0.5 * VPC.rho * (state.Va ** 2)   
        
        return baro, pitot

    def updateSensorsTrue(self, prevTrueSensors, state, dot):
        """
        Function to generate the true sensors given the current state and state derivative. 
        Sensor suite is 3-axis accelerometer, 3-axis rate gyros, 3-axis magnetometers, a barometric altimeter, 
        a pitot airspeed, and GPS with an update rate specified in the VehicleSensorConstants file. 
        For the GPS update, the previous value is returned until a new update occurs. 
        Previous value is contained within prevTrueSensors.
        """
        # Create a new container for the true sensor outputs.
        trueSensors = Sensors.vehicleSensors()
        
        # Update accelerometers (specific force) using the current state and its derivative.
        trueSensors.accel_x, trueSensors.accel_y, trueSensors.accel_z = self.updateAccelsTrue(state, dot)
        
        # Update rate gyros (body angular rates p, q, r) using the current state.
        trueSensors.gyro_x, trueSensors.gyro_y, trueSensors.gyro_z = self.updateGyrosTrue(state)
        
        # Update magnetometers by transforming the Earth's magnetic field from inertial (NED) to body frame.
        trueSensors.mag_x, trueSensors.mag_y, trueSensors.mag_z = self.updateMagsTrue(state)
        
        # Update pressure sensors:
        trueSensors.baro, trueSensors.pitot = self.updatePressureSensorsTrue(state)
        
        # Use the new GPS update only if the tick counter indicates it's time for a new measurement.
        if (self.updateTicks % self.gpsTickUpdate) == 0:
            trueSensors.gps_n, trueSensors.gps_e, trueSensors.gps_alt, trueSensors.gps_sog, trueSensors.gps_cog = self.updateGPSTrue(state, dot)
        else:
            # Retain the previous GPS true measurements (zero-order hold effect).
            trueSensors.gps_n = prevTrueSensors.gps_n
            trueSensors.gps_e = prevTrueSensors.gps_e
            trueSensors.gps_alt = prevTrueSensors.gps_alt
            trueSensors.gps_sog = prevTrueSensors.gps_sog
            trueSensors.gps_cog = prevTrueSensors.gps_cog
        
        return trueSensors


    def updateSensorsNoisy(self, 
                           trueSensors=Sensors.vehicleSensors(), 
                           noisySensors=Sensors.vehicleSensors(), 
                           sensorBiases=Sensors.vehicleSensors(), 
                           sensorSigmas=Sensors.vehicleSensors()):
        """
        Function to generate the noisy sensor data given the true sensor readings,
        the fixed biases, and the sigmas for the white noise on each sensor.
        
        For gyro and GPS, drifting biases are added.
        For GPS, if the tick counter (updateTicks) indicates it is time for a new update,
        the GPS Gauss-Markov models are updated and a new noisy measurement is generated.
        Otherwise, the previous noisy GPS values are retained (zero-order hold effect).
        
        Additionally, the white noise on GPS course over ground is scaled by the ratio of
        VPC.InitialSpeed to the current ground speed, and the resulting course is clamped to
        within +/- pi.
        
        If no GPS update has occurred, then the values for the GPS sensors are copied from the noisySensors input to the output.
        """
        # Create a new container for the noisy sensor outputs.
        noisy = Sensors.vehicleSensors()
        
        # Gyros: Beard 7.5
        gyro_drift = self.gyroGM.update()
        noisy.gyro_x = trueSensors.gyro_x + sensorBiases.gyro_x + gyro_drift + random.gauss(0, sensorSigmas.gyro_x)
        noisy.gyro_y = trueSensors.gyro_y + sensorBiases.gyro_y + gyro_drift + random.gauss(0, sensorSigmas.gyro_y)
        noisy.gyro_z = trueSensors.gyro_z + sensorBiases.gyro_z + gyro_drift + random.gauss(0, sensorSigmas.gyro_z)
        
        # Accelerometers: Beard 7.1
        noisy.accel_x = trueSensors.accel_x + sensorBiases.accel_x + random.gauss(0, sensorSigmas.accel_x)
        noisy.accel_y = trueSensors.accel_y + sensorBiases.accel_y + random.gauss(0, sensorSigmas.accel_y)
        noisy.accel_z = trueSensors.accel_z + sensorBiases.accel_z + random.gauss(0, sensorSigmas.accel_z)
        
        # Magnetometer: Beard 7.14
        noisy.mag_x = trueSensors.mag_x + sensorBiases.mag_x + random.gauss(0, sensorSigmas.mag_x)
        noisy.mag_y = trueSensors.mag_y + sensorBiases.mag_y + random.gauss(0, sensorSigmas.mag_y)
        noisy.mag_z = trueSensors.mag_z + sensorBiases.mag_z + random.gauss(0, sensorSigmas.mag_z)
        
        # Pressure Sensors: Beard 7.9
        noisy.baro = trueSensors.baro + sensorBiases.baro + random.gauss(0, sensorSigmas.baro)
        noisy.pitot = trueSensors.pitot + sensorBiases.pitot + random.gauss(0, sensorSigmas.pitot)
        
        # GPS: Beard 7.18, 7.19, 7.20
        if (self.updateTicks % self.gpsTickUpdate) == 0:
            # Update drifting biases for GPS using Gauss-Markov processes.
            gps_drift_horiz = self.gpsGM_horizontal.update()
            gps_drift_vert = self.gpsGM_vertical.update()
            
            noisy.gps_n = trueSensors.gps_n + gps_drift_horiz + random.gauss(0, sensorSigmas.gps_n)
            noisy.gps_e = trueSensors.gps_e + gps_drift_horiz + random.gauss(0, sensorSigmas.gps_e)
            noisy.gps_alt = trueSensors.gps_alt + gps_drift_vert + random.gauss(0, sensorSigmas.gps_alt)
            noisy.gps_sog = trueSensors.gps_sog + random.gauss(0, sensorSigmas.gps_sog)
            
            # For GPS course over ground, scale white noise by VPC.InitialSpeed / trueSensors.gps_sog.
            if trueSensors.gps_sog > 0.1:  # avoid division by near-zero
                cog_noise = random.gauss(0, sensorSigmas.gps_cog * (VPC.InitialSpeed / trueSensors.gps_sog))
            else:
                cog_noise = random.gauss(0, sensorSigmas.gps_cog)
            noisy.gps_cog = trueSensors.gps_cog + cog_noise
            
            # Clamp GPS COG to within +/- pi.
            noisy.gps_cog = max(min(noisy.gps_cog, math.pi), -math.pi)
        else:
            # Zero-order hold: retain previous noisy GPS measurements.
            noisy.gps_n = noisySensors.gps_n
            noisy.gps_e = noisySensors.gps_e
            noisy.gps_alt = noisySensors.gps_alt
            noisy.gps_sog = noisySensors.gps_sog
            noisy.gps_cog = noisySensors.gps_cog
        
        return noisy

    def update(self):
        """
        Wrapper function to update the Sensors (both true and noisy) using the state and its derivative 
        held within the vehicle aerodynamics model (self.aeroModel).
        """
        
        # Retrieve the current state from the aerodynamics model.
        state = self.aeroModel.getVehicleState()
        
        # Retrieve the derivative from the vehicle state object.
        dot = self.aeroModel.vehicleDynamics.getVehicleDerivative()
        
        # Generate updated true sensor measurements (ideal, noise-free).
        newTrueSensors = self.updateSensorsTrue(self.sensorsTrue, state, dot)
        self.sensorsTrue = newTrueSensors
        
        # Generate updated noisy sensor measurements by adding biases and noise.
        newNoisySensors = self.updateSensorsNoisy(newTrueSensors, self.sensorsNoisy, self.sensorsBiases, self.sensorSigmas)
        self.sensorsNoisy = newNoisySensors
        
        # Increment the tick counter.
        self.updateTicks += 1

    def setSensorsTrue(self, sensorsTrue=Sensors.vehicleSensors()):
        """
        Wrapper function to set the true sensor values.
        """
        self.sensorsTrue = sensorsTrue

    def getSensorsTrue(self):
        """
        Wrapper function to return the true sensor values.
        """
        return self.sensorsTrue

    def setSensorsNoisy(self, sensorsNoisy=Sensors.vehicleSensors()):
        """
        Wrapper function to set the noisy sensor values.
        """
        self.sensorsNoisy = sensorsNoisy

    def getSensorsNoisy(self):
        """
        Wrapper function to return the noisy sensor values.
        """
        return self.sensorsNoisy

    def reset(self):
        """
        Function to reset the module to run again. Should reset the Gauss-Markov models, re-initialize the sensor biases, 
        and reset the sensors true and noisy to pristine conditions
        """
        # Reset the Gauss-Markov processes
        self.gyroGM.reset()
        self.gpsGM_horizontal.reset()
        self.gpsGM_vertical.reset()

        # Recalculate the static sensor biases
        self.initializeBiases()

        # Reset the sensor containers to pristine (noise-free) conditions
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsNoisy = Sensors.vehicleSensors()

        # Reset the tick counter for the GPS update timing
        self.updateTicks = 0

