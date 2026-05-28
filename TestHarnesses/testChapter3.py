"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file is a test harness for the module VehicleDynamicsModel. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter3.py (from the root directory) -or-
python testChapter3.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleDynamicsModel module"""

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

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

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


#%% Derivative():
print("Beginning testing of VDM.Derivative(), subtest of [pe,pn,pd]")

cur_test = "Derivative test p_dot x dir"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 30*math.pi/180
testState.R = Rotations.euler2DCM(0.0,testState.pitch,0.0)
testState.u = 10
testDot = testVDM.derivative(testState, testFm)

print("With a velocity of u = 10 m/s, and pitch = 30deg:\n")
resultPdot = [[testDot.pn],[testDot.pe],[testDot.pd]]
expectedPdot = [[10*math.sqrt(3)/2],[0],[-10/2]]

if compareVectors(resultPdot,expectedPdot):
	evaluateTest(cur_test, True)
else:
	evaluateTest(cur_test, False)

#%%  

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""

# 1. Derivative test velocity:
cur_test = "Derivative test velocity"
testState = States.vehicleState(u=10.0, v=0.0, w=-5.0)  
testFm = Inputs.forcesMoments(Fx=5.0, Fy=0.0, Fz=-9.8)  
testDot = testVDM.derivative(testState, testFm)

# Expected results
mass = VPC.mass  
omega_cross = mm.skew(testState.p, testState.q, testState.r)
velocity_body = [[testState.u], [testState.v], [testState.w]]
omega_cross_velocity = mm.multiply(omega_cross, velocity_body)
expectedVelocities = [
    [(testFm.Fx / mass) - omega_cross_velocity[0][0]],
    [(testFm.Fy / mass) - omega_cross_velocity[1][0]],
    [(testFm.Fz / mass) - omega_cross_velocity[2][0]]
]
resultVelocities = [[testDot.u], [testDot.v], [testDot.w]]

print(f"With forces [Fx, Fy, Fz] = [{testFm.Fx}, {testFm.Fy}, {testFm.Fz}]:\n")
if compareVectors(resultVelocities, expectedVelocities):
    evaluateTest(cur_test, True)
else:
    evaluateTest(cur_test, False)

# 2. Derivative test angular rates:
cur_test = "Derivative test angular rates"
testState = States.vehicleState(p=0.1, q=0.2, r=0.3)  # Angular velocities in body frame
testFm = Inputs.forcesMoments(Mx=1.0, My=1.5, Mz=2.0)  # Torques in body frame
testDot = testVDM.derivative(testState, testFm)

# Expected results
phi, theta = testState.roll, testState.pitch
tan_theta = math.tan(theta)
sec_theta = 1 / math.cos(theta)

G = [
    [1, math.sin(phi) * tan_theta, math.cos(phi) * tan_theta],
    [0, math.cos(phi), -math.sin(phi)],
    [0, math.sin(phi) * sec_theta, math.cos(phi) * sec_theta]
]
angular_rates = [[testState.p], [testState.q], [testState.r]]
expectedAngularRates = mm.multiply(G, angular_rates)
resultAngularRates = [[testDot.roll], [testDot.pitch], [testDot.yaw]]

print(f"With angular rates [p, q, r] = [{testState.p}, {testState.q}, {testState.r}]:\n")
if compareVectors(resultAngularRates, expectedAngularRates):
    evaluateTest(cur_test, True)
else:
    evaluateTest(cur_test, False)

# 3. Derivative test angular accelerations:
cur_test = "Derivative test angular accelerations"
testState = States.vehicleState(p=0.0, q=0.0, r=0.0)  # Initial angular velocities
testFm = Inputs.forcesMoments(Mx=1.0, My=1.0, Mz=1.0)  # Torques in body frame
testDot = testVDM.derivative(testState, testFm)

# Expected results
J = VPC.Jbody
J_inv = VPC.JinvBody
omega = [[testState.p], [testState.q], [testState.r]]
omega_cross_J_omega = mm.multiply(mm.skew(testState.p, testState.q, testState.r), mm.multiply(J, omega))
torques = [[testFm.Mx], [testFm.My], [testFm.Mz]]
angular_accel = mm.subtract(mm.multiply(J_inv, torques), mm.multiply(J_inv, omega_cross_J_omega))
expectedAngularAccel = [[angular_accel[0][0]], [angular_accel[1][0]], [angular_accel[2][0]]]
resultAngularAccel = [[testDot.p], [testDot.q], [testDot.r]]

print(f"With torques [Mx, My, Mz] = [{testFm.Mx}, {testFm.My}, {testFm.Mz}]:\n")
if compareVectors(resultAngularAccel, expectedAngularAccel):
    evaluateTest(cur_test, True)
else:
    evaluateTest(cur_test, False)
    
# --- Rexp() Tests ---

print("\n---\nTesting Rexp() properties")

# Initialize VehicleDynamicsModel
testVDM = VDM.VehicleDynamicsModel()

# Helper function to check matrix orthogonality
def isOrthogonal(matrix):
    """
    Check if a matrix is orthogonal (R * R^T = I).
    """
    R_transpose = mm.transpose(matrix)
    identity = mm.multiply(R_transpose, matrix)
    expected_identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    
    for i in range(3):
        for j in range(3):
            if not isclose(identity[i][j], expected_identity[i][j]):
                return False
    return True

# Test 1: Orthogonality
cur_test = "Rexp() orthogonality test"
testDot = States.vehicleState(p=0.5, q=0.3, r=-0.2)  # Angular rates
delta_t = 0.1  # Small time step
R_exp = testVDM.Rexp(delta_t, testDot, testDot)  # Compute Rexp
evaluateTest(cur_test, isOrthogonal(R_exp))

# Test 2: Determinant = 1
cur_test = "Rexp() determinant test"
def determinant(matrix):
    """
    Compute determinant of a 3x3 matrix.
    """
    return (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
            matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
            matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]))

R_det = determinant(R_exp)
evaluateTest(cur_test, isclose(R_det, 1.0))

# Test 3: Composition consistency
cur_test = "Rexp() composition consistency test"
# Initial rotation matrix (identity)
R_initial = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

# Simulate two steps of rotation
R_step1 = testVDM.Rexp(delta_t, testDot, testDot)
R_step2 = testVDM.Rexp(delta_t, testDot, testDot)

# Combine two consecutive steps
R_combined = mm.multiply(R_step1, R_step2)

# Directly simulate the combined rotation (double delta_t)
R_direct = testVDM.Rexp(2*delta_t, testDot, testDot)

# Check if combined and direct are approximately equal
consistent = True
for i in range(3):
    for j in range(3):
        if not isclose(R_combined[i][j], R_direct[i][j]):
            consistent = False

evaluateTest(cur_test, consistent)

# --- IntegrateState Tests ---

print("\n---\nTesting IntegrateState()")

# Test 1: Position Integration
cur_test = "IntegrateState position integration"
state = States.vehicleState(pn=0.0, pe=0.0, pd=0.0)
dot = States.vehicleState(pn=10.0, pe=5.0, pd=-2.0) 
dT = 1.0
resultState = testVDM.IntegrateState(dT, state, dot)
expected_pn = state.pn + dot.pn * dT
expected_pe = state.pe + dot.pe * dT
expected_pd = state.pd + dot.pd * dT
evaluateTest(cur_test, isclose(resultState.pn, expected_pn) and
                        isclose(resultState.pe, expected_pe) and
                        isclose(resultState.pd, expected_pd))

# Test 2: Velocity Integration
cur_test = "IntegrateState velocity integration"
state = States.vehicleState(u=10.0, v=5.0, w=-2.0)
dot = States.vehicleState(u=1.0, v=0.5, w=-0.2)  
resultState = testVDM.IntegrateState(dT, state, dot)
expected_u = state.u + dot.u * dT
expected_v = state.v + dot.v * dT
expected_w = state.w + dot.w * dT
evaluateTest(cur_test, isclose(resultState.u, expected_u) and
                        isclose(resultState.v, expected_v) and
                        isclose(resultState.w, expected_w))

# Test 3: Angular Rate Integration
cur_test = "IntegrateState angular rate integration"
state = States.vehicleState(p=0.1, q=0.2, r=0.3)
dot = States.vehicleState(p=0.01, q=-0.02, r=0.03)  
resultState = testVDM.IntegrateState(dT, state, dot)
expected_p = state.p + dot.p * dT
expected_q = state.q + dot.q * dT
expected_r = state.r + dot.r * dT
evaluateTest(cur_test, isclose(resultState.p, expected_p) and
                        isclose(resultState.q, expected_q) and
                        isclose(resultState.r, expected_r))

# # Test 4: Rotation Matrix Update
# cur_test = "IntegrateState rotation matrix update"
# state = States.vehicleState(R=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
# dot = States.vehicleState(p=0.1, q=0.2, r=0.3)
# resultState = testVDM.IntegrateState(dT, state, dot)
# # Check if the updated R is orthogonal
# R_transpose = mm.transpose(resultState.R)
# R_mult = mm.multiply(R_transpose, resultState.R)
# identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
# evaluateTest(cur_test, compareVectors(R_mult[0], identity[0]) and
#                         compareVectors(R_mult[1], identity[1]) and
#                         compareVectors(R_mult[2], identity[2]))

# Test 5: Constant Variables
# cur_test = "IntegrateState constant variables"
# state = States.vehicleState(alpha=5.0, beta=3.0, Va=30.0)
# dot = States.vehicleState()  # No change in constants
# resultState = testVDM.IntegrateState(dT, state, dot)
# evaluateTest(cur_test, isclose(resultState.alpha, state.alpha) and
#                         isclose(resultState.beta, state.beta) and
#                         isclose(resultState.Va, state.Va))

# Test 6: Course Angle
# cur_test = "IntegrateState course angle"
# state = States.vehicleState()
# dot = States.vehicleState(pn=10.0, pe=5.0)
# resultState = testVDM.IntegrateState(dT, state, dot)
# expected_chi = math.atan2(dot.pe, dot.pn)
# evaluateTest(cur_test, isclose(resultState.chi, expected_chi))

print("\n---\nTesting VDM.__init__() and reset()")

cur_test = "__init__ initializes state to defaults"
testVDM = VDM.VehicleDynamicsModel()
resultState = testVDM.getVehicleState()
expectedState = States.vehicleState() 
evaluateTest(cur_test, resultState == expectedState)

cur_test = "reset() resets state and derivative to defaults"
testVDM.reset()
resultState = testVDM.getVehicleState()
resultDerivative = testVDM.getVehicleDerivative()
evaluateTest(cur_test, resultState == States.vehicleState() and resultDerivative == States.vehicleState())

print("\n---\nTesting VDM.getVehicleState() and setVehicleState()")

cur_test = "getVehicleState() and setVehicleState() work as expected"
newState = States.vehicleState(pn=100, pe=200, pd=-50, yaw=0.5, pitch=0.3, roll=0.1)
testVDM.setVehicleState(newState)
evaluateTest(cur_test, testVDM.getVehicleState() == newState)


print("\n---\nTesting VDM.getVehicleDerivative() and setVehicleDerivative()")

cur_test = "getVehicleDerivative() and setVehicleDerivative() work as expected"
newDerivative = States.vehicleState(p=0.1, q=0.2, r=0.3)
testVDM.setVehicleDerivative(newDerivative)
evaluateTest(cur_test, testVDM.getVehicleDerivative() == newDerivative)

print("\n---\nTesting VDM.Update()")

# Initialize VehicleDynamicsModel
testVDM = VDM.VehicleDynamicsModel()

# Define initial state
initial_state = States.vehicleState(
    pn=0.0, pe=0.0, pd=0.0,  # Initial position
    u=10.0, v=0.0, w=0.0,    # Initial velocity
    yaw=0.0, pitch=0.0, roll=0.0,  # Initial orientation
    p=0.0, q=0.0, r=0.0      # Initial angular rates
)
testVDM.setVehicleState(initial_state)

# Define forces and moments
forces_moments = Inputs.forcesMoments(
    Fx=10.0, Fy=0.0, Fz=-9.8,  # Forces
    Mx=0.0, My=0.0, Mz=0.0    # Moments
)

# Perform update
testVDM.Update(forces_moments)

# Retrieve updated state
updated_state = testVDM.getVehicleState()

# Expected state (manually calculated for one timestep)
delta_t = VPC.dT
expected_state = States.vehicleState(
    pn=initial_state.pn + initial_state.u * delta_t,
    pe=initial_state.pe + initial_state.v * delta_t,
    pd=initial_state.pd + initial_state.w * delta_t,
    u=initial_state.u + (forces_moments.Fx / VPC.mass) * delta_t,
    v=initial_state.v + (forces_moments.Fy / VPC.mass) * delta_t,
    w=initial_state.w + (forces_moments.Fz / VPC.mass) * delta_t,
    yaw=0.0, pitch=0.0, roll=0.0,  # No moments, so no orientation change
    p=0.0, q=0.0, r=0.0            # No moments, so no angular rate change
)

# Validate results
cur_test = "Update() test simple forward thrust"
evaluateTest(cur_test, updated_state == expected_state)

cur_test = "Update() yaw-pitch-roll alignment"

# Initialize VehicleDynamicsModel
testVDM = VDM.VehicleDynamicsModel()

# Define initial state
initial_state = States.vehicleState(
    pn=0.0, pe=0.0, pd=0.0,  
    u=10.0, v=0.0, w=0.0,    
    yaw=0.0, pitch=0.0, roll=-1.570796  # -90 degrees roll
)
testVDM.setVehicleState(initial_state)

# Define forces and moments
forces_moments = Inputs.forcesMoments(
    Fx=0.0, Fy=0.0, Fz=0.0,  
    Mx=0.0, My=0.0, Mz=0.0  
)

# Perform update
delta_t = 1.0  
testVDM.Update(forces_moments)

# Retrieve updated state
updated_state = testVDM.getVehicleState()

# Calculate expected yaw-pitch-roll manually (no forces/moments, no change in orientation)
expected_yaw = initial_state.yaw
expected_pitch = initial_state.pitch
expected_roll = initial_state.roll

# Observed values
observed_yaw, observed_pitch, observed_roll = Rotations.dcm2Euler(updated_state.R)

# Print results for debugging
print(f"Expected yaw: {expected_yaw}, Observed yaw: {observed_yaw}")
print(f"Expected pitch: {expected_pitch}, Observed pitch: {observed_pitch}")
print(f"Expected roll: {expected_roll}, Observed roll: {observed_roll}")

# Evaluate test
evaluateTest(cur_test, (
    math.isclose(expected_yaw, observed_yaw, abs_tol=1e-6) and
    math.isclose(expected_pitch, observed_pitch, abs_tol=1e-6) and
    math.isclose(expected_roll, observed_roll, abs_tol=1e-6)
))

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]