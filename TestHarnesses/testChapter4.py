"""
Test Harness for VehicleAerodynamicsModel
Author: Anderson Compalas (acompala@ucsc.edu)

This script performs directional tests and validation tests for the aerodynamics model,
including control force evaluations, gravity forces, and general force summations.
"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
from ece163.Constants import VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModel
import ece163.Modeling.WindModel as WindModel


isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

def safe_div(numerator, denominator, tol=1e-12):
    """ Perform division but handle 0/0 case and prevent division by exact zero. """
    if isclose(numerator, 0.0) and isclose(denominator, 0.0) or (numerator == 0.0 and denominator == 0.0):
        return 1.0  # Handle the special 0/0 case
    elif isclose(denominator, 0.0):
        return numerator / tol  # Prevent division by zero
    return numerator / denominator  # Normal division

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))

failed = []
passed = []
def evaluateTest(test_name, boolean):
	"""evaluateTest prints the output of a test and adds it to one of two 
	global lists, passed and failed, which can be printed later"""
	if boolean:
		print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean

def sign(x):
    return 1 if x > 0 else -1 if x < 0 else 0

# Initialize model
# Initialize vehicle model
vehicle_model = VehicleAerodynamicsModel.VehicleAerodynamicsModel()

def test_basic_functionality():
    """Tests basic getter, setter, and reset functionality."""
    initial_state = vehicle_model.getVehicleState()
    new_state = States.vehicleState()
    new_state.pn = 100.0  # Change north position
    vehicle_model.setVehicleState(new_state)
    updated_state = vehicle_model.getVehicleState()
    
    # Check if state was updated correctly
    evaluateTest("Set and Get Vehicle State", isclose(updated_state.pn, 100.0))
    
    # Reset and check if it restores initial conditions
    vehicle_model.reset()
    reset_state = vehicle_model.getVehicleState()
    evaluateTest("Reset Vehicle State", isclose(reset_state.pn, initial_state.pn))

def test_gravityForces():
    """Validates gravity force projection in the body frame."""
    testState = States.vehicleState()
    gravityForces = vehicle_model.gravityForces(testState)
    expected_Fz = 1 # Gravity should act downward. Positive in NED
    if not evaluateTest("Gravity Force Test", sign(gravityForces.Fz) == sign(expected_Fz)):
        print(sign(gravityForces.Fz))
        print(sign(expected_Fz))

def test_zeroVelocity():
    """Ensures aerodynamic forces are zero when Va = 0."""
    testState = States.vehicleState()
    aeroForces = vehicle_model.aeroForces(testState)
    evaluateTest("Zero Velocity Aero Forces", aeroForces.Fx == 0 and aeroForces.Fy == 0 and aeroForces.Fz == 0)

def test_controlForces():
    """Tests that control inputs produce expected directional moments."""
    testState = States.vehicleState()
    testState.Va = 10
    
    # Positive Velocities
    testState.x = 1
    testState.y = 1
    testState.z = 1
    control1 = Inputs.controlInputs()
    control2 = Inputs.controlInputs()
    
    # Aileron Test for Positive Roll
    control1.Aileron = 1
    control2.Aileron = -1
    fM1 = vehicle_model.controlForces(testState, control1)
    fM2 = vehicle_model.controlForces(testState, control2)
    if not evaluateTest("Aileron Directional Test", fM1.Mx > fM2.Mx):
        print(f"fM1.Mx: {fM1.Mx}")
        print(f"fM2.Mx: {fM2.Mx}")
    
    # Rudder Test for Yaw
    control1.Rudder = 1 # Rudder Deflects Left, Nose Yaws Right (negative yaw)
    control2.Rudder = -1 # Rudder Deflects Right, Nose Yaws Left (positive yaw)
    fM1 = vehicle_model.controlForces(testState, control1)
    fM2 = vehicle_model.controlForces(testState, control2)
    if not evaluateTest("Rudder Directional Test", fM1.Mz < fM2.Mz):
        print(f"fM1.Mz: {fM1.Mz}")
        print(f"fM2.Mz: {fM2.Mz}")
    
    # Elevator Test for Positive Pitch
    control1.Elevator = 1
    control2.Elevator = -1
    fM1 = vehicle_model.controlForces(testState, control1)
    fM2 = vehicle_model.controlForces(testState, control2)
    if not evaluateTest("Elevator Directional Test", fM1.My < fM2.My):
        print(f"fM1.My: {fM1.My}")
        print(f"fM2.My: {fM2.My}")

def test_aeroForces():
    """Tests that increasing velocity increases aerodynamic forces."""
    testState = States.vehicleState()
    testState.Va = 50  # Higher airspeed
    aeroForces = vehicle_model.aeroForces(testState)
    evaluateTest("Aero Forces Increase with Velocity", aeroForces.Fx < 0 and aeroForces.Fz < 0)

def test_propForces():
    """Tests that increasing throttle increases thrust."""
    Fprop_low, _ = vehicle_model.CalculatePropForces(10, 0.5)
    Fprop_high, _ = vehicle_model.CalculatePropForces(10, 1.0)
    evaluateTest("Propeller Thrust Increases with Throttle", Fprop_high > Fprop_low)

def test_updateForces():
    """Validates that updateForces correctly sums all forces."""
    testState = States.vehicleState()
    controlInputs = Inputs.controlInputs()
    totalForces = vehicle_model.updateForces(testState, controlInputs)
    gravityForces = vehicle_model.gravityForces(testState)
    aeroForces = vehicle_model.aeroForces(testState)
    controlForces = vehicle_model.controlForces(testState, controlInputs)
    
    expected_Fx = gravityForces.Fx + aeroForces.Fx + controlForces.Fx
    expected_Fy = gravityForces.Fy + aeroForces.Fy + controlForces.Fy
    expected_Fz = gravityForces.Fz + aeroForces.Fz + controlForces.Fz
    
    evaluateTest("Update Forces Summation", math.isclose(totalForces.Fx, expected_Fx, abs_tol=1e-6) and
                 math.isclose(totalForces.Fy, expected_Fy, abs_tol=1e-6) and
                 math.isclose(totalForces.Fz, expected_Fz, abs_tol=1e-6))

def test_liftCoeff():
    """Validates lift coefficient calculation for a known angle of attack."""
    alpha_test = math.radians(10) 
    CL, CD, Cm = vehicle_model.CalculateCoeff_alpha(alpha_test)
    expected_CL = VPC.CL0 + VPC.CLalpha * alpha_test
    evaluateTest("Lift Coefficient Calculation", math.isclose(CL, expected_CL, abs_tol=1e-6))
    
def test_CalculateAirspeed():
    """Validates that CalculateAirspeed correctly computes airspeed, angle of attack, and sideslip."""
    testState = States.vehicleState()
    
    # Case 1: No Wind
    testState.u, testState.v, testState.w = 25, 0, 0  # Body-frame velocity
    wind = States.windState(0, 0, 0, 0, 0, 0)  # No wind
    Va, alpha, beta = vehicle_model.CalculateAirspeed(testState, wind)
    evaluateTest("Airspeed with No Wind", isclose(Va, 25) and isclose(alpha, 0) and isclose(beta, 0))
    
    # Case 2: Pure Lateral Wind
    wind = States.windState(0, 10, 0, 0, 0, 0)  # Only crosswind
    Va, alpha, beta = vehicle_model.CalculateAirspeed(testState, wind)
    evaluateTest("Sideslip with Lateral Wind", beta > 0 and Va > 25)
    
    # Case 3: Pure Vertical Wind
    wind = States.windState(0, 0, 10, 0, 0, 0)  # Only vertical wind
    Va, alpha, beta = vehicle_model.CalculateAirspeed(testState, wind)
    evaluateTest("Angle of Attack with Vertical Wind", alpha > 0 and Va > 25)
    
wind_model = WindModel.WindModel()

def test_WindModel_Initialization():
    """Tests that WindModel initializes with default values."""
    wind_state = wind_model.getWind()
    evaluateTest("WindModel Initialization", isclose(wind_state.Wn, 0.0) and 
                 isclose(wind_state.We, 0.0) and 
                 isclose(wind_state.Wd, 0.0))

def test_WindModel_SetGet():
    """Tests setting and retrieving wind states."""
    wind_state = States.windState(5.0, -3.0, 2.0)
    wind_model.setWind(wind_state)
    retrieved_wind = wind_model.getWind()
    evaluateTest("WindModel Set/Get Wind", isclose(retrieved_wind.Wn, 5.0) and 
                 isclose(retrieved_wind.We, -3.0) and 
                 isclose(retrieved_wind.Wd, 2.0))

def test_WindModel_Reset():
    """Tests that WindModel reset restores default wind state."""
    wind_model.setWind(States.windState(2.0, 1.0, -4.0))
    wind_model.reset()
    wind_state = wind_model.getWind()
    evaluateTest("WindModel Reset", isclose(wind_state.Wn, 0.0) and 
                 isclose(wind_state.We, 0.0) and 
                 isclose(wind_state.Wd, 0.0))

def test_WindModel_Update():
    """Tests statistical properties of WindModel.Update()."""
    num_samples = 10000
    wind_samples = []
    
    for _ in range(num_samples):
        wind_model.Update()
        wind_state = wind_model.getWind()
        wind_samples.append(wind_state.Wv)
    
    mean_wind = sum(wind_samples) / num_samples
    std_dev_wind = (sum([(w - mean_wind) ** 2 for w in wind_samples]) / num_samples) ** 0.5
    
    expected_sigma_v = 1.06 
    
    evaluateTest("WindModel Std Dev Test", isclose(std_dev_wind, expected_sigma_v))

def test_CreateDrydenTransferFns(test_name, Va, dryden_params):
    """Generic function to test CreateDrydenTransferFns with different conditions."""
    wind_model = WindModel.WindModel(drydenParameters=dryden_params, Va=Va)

    # Retrieve transfer functions
    Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w = wind_model.getDrydenTransferFns()

    # Expected values from grading script
    expected_values = {
        "Phi_u_0_0": 1.000000e+00,
        "H_u_0_0": 1.000000e+00,
        "Phi_v_0_0": 1.000000e+00,
        "Phi_v_1_1": 1.000000e+00,
        "H_v_0_0": 1.000000e+00,
        "H_v_0_1": 1.000000e+00,
        "Phi_w_0_0": 1.000000e+00,
        "Phi_w_1_1": 1.000000e+00,
        "H_w_0_0": 1.000000e+00,
        "H_w_0_1": 1.000000e+00
    }

    observed_values = {
        "Phi_u_0_0": Phi_u[0][0],
        "H_u_0_0": H_u[0][0],
        "Phi_v_0_0": Phi_v[0][0],
        "Phi_v_1_1": Phi_v[1][1],
        "H_v_0_0": H_v[0][0],
        "H_v_0_1": H_v[0][1],
        "Phi_w_0_0": Phi_w[0][0],
        "Phi_w_1_1": Phi_w[1][1],
        "H_w_0_0": H_w[0][0],
        "H_w_0_1": H_w[0][1]
    }

    # Check if values match expected
    test_failed = False
    error_messages = []

    for key in expected_values:
        if not math.isclose(observed_values[key], expected_values[key], abs_tol=1e-6):
            test_failed = True
            error_messages.append(f"   {key}: exp= {expected_values[key]:.6e} != obs= {observed_values[key]:.6e} "
                                  f"({expected_values[key] - observed_values[key]:.6e})")

    if test_failed:
        failed.append(test_name)
        print(f"\nFAILED {test_name}")
        print(f"Dryden Parameters: {dryden_params}")
        print(f"Va: {Va}, Lu: {dryden_params.Lu}")
        print(f"Value: {safe_div(Va, dryden_params.Lu)}")
        for msg in error_messages:
            print(msg)
    else:
        passed.append(test_name)
        print(f"   passed {test_name}")
        

def test_CalculateAirspeed():
    """Validates that CalculateAirspeed correctly computes airspeed, angle of attack, and sideslip."""
    testState = States.vehicleState()
    
    # Case 1: No Wind
    testState.u, testState.v, testState.w = 25, 0, 0  # Body-frame velocity
    wind = States.windState(0, 0, 0, 0, 0, 0)  # No wind
    Va, alpha, beta = vehicle_model.CalculateAirspeed(testState, wind)
    evaluateTest("Airspeed with No Wind", isclose(Va, 25) and isclose(alpha, 0) and isclose(beta, 0))
    
    # Case 2: Pure Lateral Wind
    wind = States.windState(0, 10, 0, 0, 0, 0)  # Only crosswind
    Va, alpha, beta = vehicle_model.CalculateAirspeed(testState, wind)
    evaluateTest("Sideslip with Lateral Wind", beta > 0 and Va > 25)
    
    # Case 3: Pure Vertical Wind
    wind = States.windState(0, 0, 10, 0, 0, 0)  # Only vertical wind
    Va, alpha, beta = vehicle_model.CalculateAirspeed(testState, wind)
    evaluateTest("Angle of Attack with Vertical Wind", alpha > 0 and Va > 25)



# Run Tests
test_basic_functionality()
test_gravityForces()
test_zeroVelocity()
test_controlForces()
test_aeroForces()
test_propForces()
# test_updateForces()
test_liftCoeff()
    
# Run the test for CalculateAirspeed
# test_CalculateAirspeed()

# Zero vector tests
test_CreateDrydenTransferFns("wm_createTransferFunctions_5_25_DrydenNoWind", 25, Inputs.drydenParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
test_CreateDrydenTransferFns("wm_createTransferFunctions_11_100_DrydenNoWind", 100, Inputs.drydenParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
test_CreateDrydenTransferFns("wm_createTransferFunctions_17_0.001_DrydenNoWind", 0.001, Inputs.drydenParameters(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


# More Tests
test_WindModel_Initialization()
test_WindModel_SetGet()
test_WindModel_Reset()
test_CalculateAirspeed()

testState = States.vehicleState()
gravity_forces = vehicle_model.gravityForces(testState)
print(gravity_forces)

def test_initial_nose_dive():
    """Tests if the initial total forces create excessive nose-down pitching moment (My)."""
    testState = States.vehicleState()  # Initialize default state (steady flight)
    
    #  (Va = 25 m/s, no wind, no pitch)
    testState.Va = 25.0
    testState.u = 25.0  # All forward motion
    testState.v = 0.0   # No sideslip
    testState.w = 0.0   # No vertical velocity
    testState.alpha = 0.0  # No angle of attack
    testState.beta = 0.0   # No sideslip

    controlInputs = Inputs.controlInputs()  # Neutral controls
    
    # Compute forces
    totalForces = vehicle_model.updateForces(testState, controlInputs)

    # Print debug information
    print("\n=== Debugging Initial Nosedive ===")
    print(f"Total Forces: Fx={totalForces.Fx}, Fy={totalForces.Fy}, Fz={totalForces.Fz}")
    print(f"Total Moments: Mx={totalForces.Mx}, My={totalForces.My}, Mz={totalForces.Mz}")
    print(f"Initial Airspeed: {testState.Va}, Alpha: {math.degrees(testState.alpha)} deg, Beta: {math.degrees(testState.beta)} deg")

    # Evaluate the test
    if totalForces.My < 0:  
        evaluateTest("Initial Pitching Moment", False)  # Nose-down
    else:
        evaluateTest("Initial Pitching Moment", True)  # No nosedive tendency


# Run the new test
test_initial_nose_dive()


print(f"\nTest Summary: Passed {len(passed)}/{len(passed) + len(failed)} tests")
