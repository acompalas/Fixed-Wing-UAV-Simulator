"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains classes to calculate the closed loop control gains for the aircraft.
"""

import math
import sys
import ece163.Containers.Inputs as Inputs
from ..Containers import Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
from ece163.Controls.VehicleEstimator import VehicleEstimator
import ece163.Sensors.SensorsModel as SensorsModel

class PDControl:
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Initializes the PD control gains and limits."""
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.prevError = 0.0
        self.isSaturated = False

    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Sets new PD gains and limits for in-flight adjustments."""
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        """
        Updates the PD controller output based on the input command and feedback signal.
        """
        # 1. Calculate the error
        error = command - current
        
        # 2. Compute the PD components
        proportional = self.kp * error
        derivative_term = -self.kd * derivative
        
        # 3. Sum up the terms and add trim
        u = proportional + derivative_term + self.trim

        # 4. Apply saturation limits
        if u > self.highLimit:
            u = self.highLimit
            self.isSaturated = True
        elif u < self.lowLimit:
            u = self.lowLimit
            self.isSaturated = True
        else:
            self.isSaturated = False

        # 5. Store the current error for the next iteration
        self.prevError = error

        return u

    def __repr__(self):
        return (f"PDControl(kp={self.kp}, kd={self.kd}, trim={self.trim}, "
                f"lowLimit={self.lowLimit}, highLimit={self.highLimit}, "
                f"prevError={self.prevError})")


class PIControl:
    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Initializes the PI control gains and limits."""
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.prevError = 0.0
        self.isSaturated = False

    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Sets new PI gains and limits for in-flight adjustments."""
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

    def resetIntegrator(self):
        """Resets the integral accumulator and previous error to zero."""
        self.accumulator = 0.0
        self.prevError = 0.0

    def Update(self, command=0.0, current=0.0):
        """Updates the PI controller output based on the input command and feedback signal."""
        # 1. Calculate the error
        error = command - current

        # 2. Update the accumulator using trapezoidal integration
        if not self.isSaturated:
            self.accumulator += 0.5 * (error + self.prevError) * self.dT

        # 3. Compute the PI components
        proportional = self.kp * error
        integral = self.ki * self.accumulator

        # 4. Sum up the terms and add trim
        u = proportional + integral + self.trim

        # 5. Apply saturation limits
        if u > self.highLimit:
            u = self.highLimit
            self.isSaturated = True
        elif u < self.lowLimit:
            u = self.lowLimit
            self.isSaturated = True
        else:
            self.isSaturated = False

        # 6. Store the current error for the next iteration
        self.prevError = error

        return u

    def __repr__(self):
        return (f"PIControl(dT={self.dT}, kp={self.kp}, ki={self.ki}, trim={self.trim}, "
                f"lowLimit={self.lowLimit}, highLimit={self.highLimit}, "
                f"accumulator={self.accumulator}, prevError={self.prevError})")

class PIDControl:
    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Initializes the PID control gains and limits."""
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.prevError = 0.0
        self.isSaturated = False  

    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        """Sets new PID gains and limits for in-flight adjustments."""
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
            
    def resetIntegrator(self):
        """Resets the integral accumulator and previous error to zero."""
        self.accumulator = 0.0
        self.prevError = 0.0

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        """
        Updates the PID controller output based on the input command and feedback signal.
        """
        # 1. Calculate the error
        error = command - current

        # 2. Improved Anti-Windup: Only integrate if NOT saturated
        if not self.isSaturated:
            self.accumulator += 0.5 * (error + self.prevError) * self.dT  # Trapezoidal Integration

        # 3. Compute PID components
        proportional = self.kp * error
        integral = self.ki * self.accumulator
        derivative_term = -self.kd * derivative

        # 4. Correct PID formula with Trim
        u = proportional + integral + derivative_term + self.trim

        # 5. Improved Saturation Check and Anti-Windup Handling
        if u > self.highLimit:
            u = self.highLimit
            self.isSaturated = True
        elif u < self.lowLimit:
            u = self.lowLimit
            self.isSaturated = True
        else:
            self.isSaturated = False

        # 6. Store current error for next iteration
        self.prevError = error

        return u

    def __repr__(self):
        return (f"PIDControl(dT={self.dT}, kp={self.kp}, kd={self.kd}, ki={self.ki}, "
                f"trim={self.trim}, lowLimit={self.lowLimit}, highLimit={self.highLimit}, "
                f"accumulator={self.accumulator}, prevError={self.prevError})")


class VehicleClosedLoopControl:
    def __init__(self, dT=VPC.dT, rudderControlSource="SIDESLIP", useSensors=False, useEstimator=False):
        """Initializes the closed-loop control system with all controllers and required components."""
    
        # 1. Create the VehicleAerodynamicsModel (the plant)
        self.vehicleAerodynamicsModel = VehicleAerodynamicsModule.VehicleAerodynamicsModel(dT)

        # 2. Create containers for control parameters and inputs
        self.controlGains = Controls.controlGains()            # Stores KP, KI, KD for controllers
        self.trimInputs = Inputs.controlInputs()               # Stores trim control inputs
        self.controlSurfaces = Inputs.controlInputs()          # Stores outputs to the plant
        
        # 3. Store the time step
        self.dT = dT

        # 4. Set initial mode (AltitudeState): HOLDING
        self.mode = Controls.AltitudeStates.HOLDING

        # 5. Instantiate 7 feedback controllers based on Beard’s recipe:
        self.rollFromCourse = PIControl(dT)                  # PIControl for Roll
        self.rudderFromSideslip = PIControl(dT)              # PIControl for Sideslip
        self.throttleFromAirspeed = PIControl(dT)            # PIControl for Airspeed (Throttle)
        self.pitchFromAltitude = PIControl(dT)               # PIControl for Altitude
        self.pitchFromAirspeed = PIControl(dT)               # PIControl for Airspeed (Elevator)

        self.elevatorFromPitch = PDControl()                 # PDControl for Pitch Angle
        self.aileronFromRoll = PIDControl(dT)                # PIDControl for Roll Angle
        
        # 6. Store additional flags 
        self.rudderControlSource = rudderControlSource
        self.useSensors = useSensors
        self.useEstimator = useEstimator
        
        # 7. Pass the already-created vehicle aerodynamics model to the SensorsModel.
        if self.useSensors == True:
            self.sensorsModel = SensorsModel.SensorsModel(aeroModel=self.vehicleAerodynamicsModel)
            
        # 8. Initialize the VehicleEstimator if enabled
        if self.useEstimator == True:
            self.vehicleEstimator = VehicleEstimator(dT=self.dT, sensorsModel=self.sensorsModel)

    def setControlGains(self, controlGains=Controls.controlGains()):
        """Sets PID gains and trims for all controllers based on provided controlGains and VPC."""
        self.controlGains = controlGains  # Store control gains for future reference
        
        # Roll (Course to Roll) - PIControl
        self.rollFromCourse.setPIGains(
            kp=controlGains.kp_course, 
            ki=controlGains.ki_course, 
            trim=0.0,
            lowLimit=-math.radians(VPC.bankAngleLimit), 
            highLimit=math.radians(VPC.bankAngleLimit)
        )
        
        # Sideslip (Rudder Control) - PIControl
        self.rudderFromSideslip.setPIGains(
            kp=controlGains.kp_sideslip, 
            ki=controlGains.ki_sideslip, 
            trim=self.trimInputs.Rudder,  
            lowLimit=VPC.minControls.Rudder, 
            highLimit=VPC.maxControls.Rudder
        )
        
        # Airspeed (Throttle Control) - PIControl
        self.throttleFromAirspeed.setPIGains(
            kp=controlGains.kp_SpeedfromThrottle, 
            ki=controlGains.ki_SpeedfromThrottle, 
            trim=self.trimInputs.Throttle, 
            lowLimit=VPC.minControls.Throttle, 
            highLimit=VPC.maxControls.Throttle
        )
        
        # Altitude (Pitch from Altitude) - PIControl
        self.pitchFromAltitude.setPIGains(
            kp=controlGains.kp_altitude, 
            ki=controlGains.ki_altitude, 
            trim=0.0,
            lowLimit=-math.radians(VPC.pitchAngleLimit), 
            highLimit=math.radians(VPC.pitchAngleLimit)
        )
        
        # Airspeed (Pitch from Elevator) - PIControl
        self.pitchFromAirspeed.setPIGains(
            kp=controlGains.kp_SpeedfromElevator, 
            ki=controlGains.ki_SpeedfromElevator, 
            trim=0.0,
            lowLimit=-math.radians(VPC.pitchAngleLimit), 
            highLimit=math.radians(VPC.pitchAngleLimit)
        )
        
        # Pitch Angle (Elevator) - PDControl
        self.elevatorFromPitch.setPDGains(
            kp=controlGains.kp_pitch, 
            kd=controlGains.kd_pitch, 
            trim=self.trimInputs.Elevator,  
            lowLimit=VPC.minControls.Elevator, 
            highLimit=VPC.maxControls.Elevator
        )
        
        # Roll Angle (Aileron) - PIDControl
        self.aileronFromRoll.setPIDGains(
            kp=controlGains.kp_roll, 
            ki=controlGains.ki_roll, 
            kd=controlGains.kd_roll, 
            trim=self.trimInputs.Aileron, 
            lowLimit=VPC.minControls.Aileron, 
            highLimit=VPC.maxControls.Aileron
        )


    def getControlGains(self):
        """Returns the current control gains."""
        return self.controlGains

    def getVehicleState(self):
        """Returns the current vehicle state from the aerodynamics model."""
        return self.vehicleAerodynamicsModel.getVehicleState()

    def setTrimInputs(self, trimInputs=Inputs.controlInputs()):
        """Sets the trim inputs for the vehicle."""
        self.trimInputs = trimInputs

    def getTrimInputs(self):
        """Returns the current trim inputs."""
        return self.trimInputs

    def setVehicleState(self, state):
        """Sets the vehicle state in the aerodynamics model."""
        self.vehicleAerodynamicsModel.setVehicleState(state)

    def getVehicleControlSurfaces(self):
        """Returns the current control surfaces outputs."""
        return self.controlSurfaces

    def getVehicleAerodynamicsModel(self):
        """Returns the vehicle aerodynamics model."""
        return self.vehicleAerodynamicsModel

    def getSensorsModel(self):
        """Returns the sensors model, if available."""
        if self.useSensors:
            return self.sensorsModel
        return None

    def getVehicleEstimator(self):
        """Returns the vehicle state estimator, if available."""
        return self.vehicleEstimator if self.useEstimator else None

    def reset(self):
        """Resets all controllers' integrators and the vehicle aerodynamics model."""
        
        # Reset integrators for the six controllers that have them (PI and PID controllers)
        self.rollFromCourse.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.aileronFromRoll.resetIntegrator()

        # Reset the VehicleAerodynamicsModel instance
        self.vehicleAerodynamicsModel.reset()
        
        # Reset SensorsModel
        if self.useSensors:
            self.sensorsModel.reset()
            
        # Reset Estimators
        if self.useEstimator:
            self.vehicleEstimator.reset()
            
    def update(self, referenceCommands=Controls.referenceCommands()):
        """Performs one update cycle of the closed-loop control system."""
        # Compute control inputs using the reference commands and current vehicle state
        current_state = self.vehicleAerodynamicsModel.getVehicleState()
        control_inputs = self.UpdateControlCommands(referenceCommands, current_state)
        
        # Send control inputs to the vehicle aerodynamics model to update the state
        self.vehicleAerodynamicsModel.Update(control_inputs)
        
        if self.useSensors:
            self.sensorsModel.update()
            
        if self.useEstimator:
            estimated_state = self.vehicleEstimator.estimatedState
            self.controlSurfaces = self.UpdateControlCommands(
                referenceCommands, estimated_state
            )

        if self.useEstimator:
            self.vehicleEstimator.Update()
            
        # Return the updated control inputs for reference
        return control_inputs

    def UpdateControlCommands(self, referenceCommands, state):
        """Computes and assembles control inputs based on current state and reference commands."""
        
        # 1. Handle Course Error (Wrap to +-pi)
        course_error = referenceCommands.commandedCourse - state.chi
        if course_error > math.pi:
            state.chi += 2 * math.pi
        elif course_error < -math.pi:
            state.chi -= 2 * math.pi

        # Special Case for chi = 0 and commandedCourse = ±π
        if abs(referenceCommands.commandedCourse) == math.pi and state.chi == 0:
            rollCommand = -self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi)
        else:
            rollCommand = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi)
            
        aileronCommand = self.aileronFromRoll.Update(rollCommand, state.roll, state.p)

        # 2. Compute Roll Command from Course
        # rollCommand = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi) # PI Control
        # aileronCommand = self.aileronFromRoll.Update(rollCommand, state.roll, state.p) # PID Control
        
        # 3. Compute Rudder Command from Sideslip
        rudderCommand = self.rudderFromSideslip.Update(0.0, state.beta)

        # 4. Determine Mode Based on Altitude (State Machine)
        commandedAltitude = referenceCommands.commandedAltitude
        altitude = -state.pd
        upper_thresh = commandedAltitude + VPC.altitudeHoldZone
        lower_thresh = commandedAltitude - VPC.altitudeHoldZone

        if altitude > upper_thresh:
            self.mode = Controls.AltitudeStates.DESCENDING
            self.pitchFromAirspeed.resetIntegrator()
        elif altitude < lower_thresh:
            self.mode = Controls.AltitudeStates.CLIMBING
            self.pitchFromAirspeed.resetIntegrator()
        else:
            self.mode = Controls.AltitudeStates.HOLDING
            self.pitchFromAltitude.resetIntegrator()

        # 5. Compute Pitch and Throttle Commands Based on Mode
        if self.mode == Controls.AltitudeStates.HOLDING:
            pitchCommand = self.pitchFromAltitude.Update(commandedAltitude, altitude)
            throttleCommand = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
        elif self.mode == Controls.AltitudeStates.CLIMBING:
            pitchCommand = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
            throttleCommand = VPC.maxControls.Throttle
        else:  # DESCENDING
            pitchCommand = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
            throttleCommand = VPC.minControls.Throttle

        # 6. Compute Elevator Command from Pitch
        elevatorCommand = self.elevatorFromPitch.Update(pitchCommand, state.pitch, state.q)

        # 7. Update Reference Commands for GUI
        referenceCommands.commandedPitch = pitchCommand
        referenceCommands.commandedRoll = rollCommand

        # 8. Assemble Outputs into controlInputs Container
        control_inputs = Inputs.controlInputs(
            Throttle=throttleCommand,
            Aileron=aileronCommand,
            Elevator=elevatorCommand,
            Rudder=rudderCommand
        )

        return control_inputs
    