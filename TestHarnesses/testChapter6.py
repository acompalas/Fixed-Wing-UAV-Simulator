"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains a simple test harness with simple tests for every function.
"""

import sys
import math
import matplotlib.pyplot as plt

# Append ece163 modules
sys.path.append("..")  

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehicleClosedLoopControl as VCLC
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Containers.Controls as Controls
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Controls.VehicleClosedLoopControl import PDControl, PIControl, PIDControl, VehicleClosedLoopControl

# Helper function to check numeric and non-numeric equality
def evaluateTest(test_name, observed, expected):
    """Evaluates the test by checking equality for non-numerics and approximate equality for numerics."""
    if isinstance(observed, (int, float)) and isinstance(expected, (int, float)):
        passed_test = math.isclose(observed, expected, abs_tol=1e-12)
    else:
        passed_test = (observed == expected)

    if passed_test:
        print(f"   passed {test_name}")
        passed.append(test_name)
    else:
        print(f"   failed {test_name}")
        print(f"      Expected: {expected}")
        print(f"      Observed: {observed}")
        failed.append(test_name)

    return passed_test

# Global lists for test results
passed = []
failed = []

print(f"PDControl from {PDControl.__module__}")
print(f"PIControl from {PIControl.__module__}")
print(f"PIDControl from {PIDControl.__module__}")

### TESTS FOR CONTROLLERS (PD, PI, PID) ###

# Test 1: PDControl Basic Functionality
def test_PDControl_basic():
    """Tests PD controller output for a simple input."""
    pd = PDControl(kp=1.0, kd=0.5, trim=0.2, lowLimit=-10.0, highLimit=10.0)
    output = pd.Update(command=1.0, current=0.5, derivative=0.1)
    expected = 0.2 + 1.0*(0.5) + 0.5*(0.1)
    evaluateTest("PDControl Basic Output", output, expected)

# Test 2: PIControl Basic Functionality (Trapezoidal Integration)
def test_PIControl_basic():
    """Tests PI controller output with simple input and accumulated error."""
    pi = PIControl(kp=1.0, ki=0.5, trim=0.1, lowLimit=-10.0, highLimit=10.0)
    pi.accumulator = 0.2
    pi.prevError = 0.0  
    output = pi.Update(command=2.0, current=1.0)
    current_error = 2.0 - 1.0  # error = 1.0
    expected_accumulator = 0.2 + 0.5 * (current_error + 0.0) * VPC.dT 
    expected = 0.1 + 1.0 * current_error + 0.5 * expected_accumulator
    
    evaluateTest("PIControl Basic Output", output, expected)

# Test 3: PIDControl Basic Functionality (Trapezoidal Integration)
def test_PIDControl_basic():
    """Tests PID controller output for simple inputs."""
    pid = PIDControl(kp=1.0, ki=0.5, kd=0.2, trim=0.1, lowLimit=-10.0, highLimit=10.0)
    pid.accumulator = 0.3
    pid.prevError = 0.0  
    output = pid.Update(command=3.0, current=2.5, derivative=0.2)
    current_error = 3.0 - 2.5  # error = 0.5
    expected_accumulator = 0.3 + 0.5 * (current_error + 0.0) * VPC.dT  
    expected = (
        0.1
        + 1.0 * current_error  # proportional
        + 0.5 * expected_accumulator  # integral
        + 0.2 * 0.2  # derivative
    )
    
    evaluateTest("PIDControl Basic Output", output, expected)

### TESTS FOR VehicleClosedLoopControl 

# Test 4: VehicleClosedLoopControl Initialization
def test_VCLC_initialization():
    """Tests correct initialization of VehicleClosedLoopControl."""
    vclc = VehicleClosedLoopControl()
    evaluateTest("VCLC Initialization - Mode", vclc.mode, Controls.AltitudeStates.HOLDING)
    evaluateTest("VCLC Initialization - Roll Controller Type", type(vclc.rollFromCourse), PIControl)
    evaluateTest("VCLC Initialization - Elevator Controller Type", type(vclc.elevatorFromPitch), PDControl)
    evaluateTest("VCLC Initialization - Aileron Controller Type", type(vclc.aileronFromRoll), PIDControl)

# Test 5: VehicleClosedLoopControl setControlGains
def test_VCLC_setControlGains():
    """Tests that setControlGains correctly assigns gains to each controller."""
    control_gains = Controls.controlGains(
        kp_roll=1.0, kd_roll=0.2, ki_roll=0.3,
        kp_sideslip=0.5, ki_sideslip=0.1,
        kp_course=1.5, ki_course=0.7,
        kp_pitch=0.8, kd_pitch=0.4,
        kp_altitude=0.7, ki_altitude=0.2,
        kp_SpeedfromThrottle=0.9, ki_SpeedfromThrottle=0.3,
        kp_SpeedfromElevator=0.95, ki_SpeedfromElevator=0.25
    )
    
    vclc = VehicleClosedLoopControl()
    vclc.setControlGains(control_gains)
    
    # Roll Controller Gains
    evaluateTest("VCLC Gains - Roll KP", vclc.rollFromCourse.kp, 1.5)  # course → roll
    evaluateTest("VCLC Gains - Roll KI", vclc.rollFromCourse.ki, 0.7)  

    # Sideslip Controller Gains
    evaluateTest("VCLC Gains - Sideslip KP", vclc.rudderFromSideslip.kp, 0.5)
    evaluateTest("VCLC Gains - Sideslip KI", vclc.rudderFromSideslip.ki, 0.1)

    # Throttle Controller Gains (Airspeed from Throttle)
    evaluateTest("VCLC Gains - Throttle KP", vclc.throttleFromAirspeed.kp, 0.9)
    evaluateTest("VCLC Gains - Throttle KI", vclc.throttleFromAirspeed.ki, 0.3)

    # Altitude Controller Gains
    evaluateTest("VCLC Gains - Altitude KP", vclc.pitchFromAltitude.kp, 0.7)
    evaluateTest("VCLC Gains - Altitude KI", vclc.pitchFromAltitude.ki, 0.2)

    # Airspeed from Elevator Gains
    evaluateTest("VCLC Gains - Elevator Airspeed KP", vclc.pitchFromAirspeed.kp, 0.95)
    evaluateTest("VCLC Gains - Elevator Airspeed KI", vclc.pitchFromAirspeed.ki, 0.25)

    # Elevator (Pitch Angle) Gains
    evaluateTest("VCLC Gains - Pitch KP", vclc.elevatorFromPitch.kp, 0.8)
    evaluateTest("VCLC Gains - Pitch KD", vclc.elevatorFromPitch.kd, 0.4)

    # Aileron (Roll Angle) Gains
    evaluateTest("VCLC Gains - Aileron KP", vclc.aileronFromRoll.kp, 1.0)
    evaluateTest("VCLC Gains - Aileron KI", vclc.aileronFromRoll.ki, 0.3)
    evaluateTest("VCLC Gains - Aileron KD", vclc.aileronFromRoll.kd, 0.2)

# Test 6: VehicleClosedLoopControl reset
def test_VCLC_reset():
    """Tests that reset properly clears integrators and resets the aerodynamics model."""
    vclc = VehicleClosedLoopControl()

    # Set accumulators to non-zero values
    vclc.rollFromCourse.accumulator = 5.0
    vclc.rudderFromSideslip.accumulator = 3.0
    vclc.throttleFromAirspeed.accumulator = 4.0
    vclc.pitchFromAltitude.accumulator = 2.0
    vclc.pitchFromAirspeed.accumulator = 1.0
    vclc.aileronFromRoll.accumulator = 6.0
    
    vclc.reset()

    # Verify accumulators are zero after reset
    evaluateTest("VCLC Reset - Roll Accumulator", vclc.rollFromCourse.accumulator, 0.0)
    evaluateTest("VCLC Reset - Sideslip Accumulator", vclc.rudderFromSideslip.accumulator, 0.0)
    evaluateTest("VCLC Reset - Throttle Accumulator", vclc.throttleFromAirspeed.accumulator, 0.0)
    evaluateTest("VCLC Reset - Altitude Accumulator", vclc.pitchFromAltitude.accumulator, 0.0)
    evaluateTest("VCLC Reset - Airspeed Accumulator", vclc.pitchFromAirspeed.accumulator, 0.0)
    evaluateTest("VCLC Reset - Aileron Accumulator", vclc.aileronFromRoll.accumulator, 0.0)

# Test 7: Tests for getters and setters
def test_getters_and_setters():
    """Tests that setters and getters are functional."""
    vclc = VehicleClosedLoopControl()
    
    # Test set and get Trim Inputs
    trim_inputs = Inputs.controlInputs(0.1, 0.2, 0.3, 0.4)
    vclc.setTrimInputs(trim_inputs)
    evaluateTest("getTrimInputs", vclc.getTrimInputs(), trim_inputs)
    
    # Test get Control Gains
    control_gains = Controls.controlGains(
        kp_roll=1.1, kd_roll=0.9, ki_roll=0.3,
        kp_sideslip=0.5, ki_sideslip=0.1,
        kp_course=1.5, ki_course=0.7,
        kp_pitch=0.8, kd_pitch=0.4,
        kp_altitude=0.7, ki_altitude=0.2,
        kp_SpeedfromThrottle=0.9, ki_SpeedfromThrottle=0.3,
        kp_SpeedfromElevator=0.95, ki_SpeedfromElevator=0.25
    )
    vclc.setControlGains(control_gains)
    evaluateTest("getControlGains", vclc.getControlGains(), control_gains)

    # Test get Vehicle Aerodynamics Model
    evaluateTest("getVehicleAerodynamicsModel", type(vclc.getVehicleAerodynamicsModel()).__name__, "VehicleAerodynamicsModel")
    
    # Test get Sensors Model (default should be None until further labs)
    evaluateTest("getSensorsModel", vclc.getSensorsModel(), None)

    # Test get Vehicle Estimator (default should be None until further labs)
    evaluateTest("getVehicleEstimator", vclc.getVehicleEstimator(), None)
    

# Test 8: Test UpdateControlCommands    
def test_UpdateControlCommands_modes():
    """Tests mode switching (CLIMBING, DESCENDING, HOLDING) based on altitude thresholds."""
    vclc = VehicleClosedLoopControl()
    refCmd = Controls.referenceCommands(airspeedCommand=VPC.InitialSpeed)
    state = States.vehicleState()

    # Test CLIMBING
    state.pd = -(refCmd.commandedAltitude - VPC.altitudeHoldZone - 1)
    vclc.UpdateControlCommands(refCmd, state)
    evaluateTest("UpdateControlCommands Mode - CLIMBING", vclc.mode, Controls.AltitudeStates.CLIMBING)

    # Test DESCENDING
    state.pd = -(refCmd.commandedAltitude + VPC.altitudeHoldZone + 1)
    vclc.UpdateControlCommands(refCmd, state)
    evaluateTest("UpdateControlCommands Mode - DESCENDING", vclc.mode, Controls.AltitudeStates.DESCENDING)

    # Test HOLDING
    state.pd = -refCmd.commandedAltitude
    vclc.UpdateControlCommands(refCmd, state)
    evaluateTest("UpdateControlCommands Mode - HOLDING", vclc.mode, Controls.AltitudeStates.HOLDING)


### RUN ALL TESTS 

print("\n### RUNNING CONTROLLER TESTS ###")
test_PDControl_basic()
test_PIControl_basic()
test_PIDControl_basic()

print("\n### RUNNING VEHICLE CLOSED LOOP CONTROL TESTS ###")
test_VCLC_initialization()
test_VCLC_setControlGains()
test_VCLC_reset()

print("\n### TESTS FOR SETTERS AND GETTERS ###")
test_getters_and_setters()

print("\n### TESTING UpdateControlCommands ###")
test_UpdateControlCommands_modes()

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")

if failed:
    print(f"Failed {len(failed)}/{total} tests:")
    for test in failed:
        print(f"   {test}")

print(math.exp(-0.01/1e6))