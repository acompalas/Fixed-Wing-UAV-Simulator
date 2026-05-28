"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter2.py (from the root directory) -or-
python testChapter2.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the Rotations module"""

"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains tests for each of the functions in Lab 0 .
"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG

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


#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm yaw test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(90*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[-1],[0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


#%%  

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""

cur_test = "Euler2dcm identity test"
# rotating [1,0,0] by (0,0,0) should return [1,0,0]
R = Rotations.euler2DCM(0, 0, 0)
orig_vec = [[1], [0], [0]]
expected_vec = [[1], [0], [0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec)):
	print(f"Expected: {expected_vec}, Actual: {actual_vec}")

cur_test = "Euler2dcm non-trivial angles test"
# rotating [1,0,0] by (45, 30, 60) degrees
R = Rotations.euler2DCM(45*math.pi/180, 30*math.pi/180, 60*math.pi/180)
orig_vec = [[1], [0], [0]]
expected_vec = [[math.sqrt(6)/4], [(math.sqrt(6)-2*math.sqrt(2))/8], [(math.sqrt(2)+2*math.sqrt(6))/8]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec)):
	print(f"Expected: {expected_vec}, Actual: {actual_vec}")

#%% DCM2Euler():
print("Beginning testing of Rotations.DCM2Euler()")

cur_test = "DCM2Euler identity test"
# converting identity matrix back to Euler angles should return (0,0,0)
DCM = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
expected_vec = [[0], [0], [0]] 
actual_vec = [[val] for val in Rotations.dcm2Euler(DCM)]
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec)):
	print(f"Expected: {expected_vec}, Actual: {actual_vec}")

cur_test = "DCM2Euler non-trivial test"
# converting non-trivial DCM back to Euler angles
sqrt2 = math.sqrt(2)
sqrt3 = math.sqrt(3)
sqrt6 = math.sqrt(6)
DCM = [
    [sqrt6 / 4, sqrt6 / 4, -1 / 2],
    [(sqrt6 - 2 * sqrt2) / 8, (sqrt6 + 2 * sqrt2) / 8, 3 / 4],
    [(sqrt2 + 2 * sqrt6) / 8, (sqrt2 - 2 * sqrt6) / 8, sqrt3 / 4]
]
expected_vec = [[45*math.pi/180], [30*math.pi/180], [60*math.pi/180]]
actual_vec = [[val] for val in Rotations.dcm2Euler(DCM)]
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec)):
	print(f"Expected: {expected_vec}, Actual: {actual_vec}")

#%% NED to ENU:
print("Beginning testing of Rotations.NED2ENU()")

cur_test = "ned2enu simple test"
ned_point = [1, 2, 3]  # Single NED point
expected_enu = [2, 1, -3]  # Expected ENU point after conversion
actual_enu = Rotations.ned2enu(ned_point)

if not evaluateTest(cur_test, actual_enu == expected_enu):
    print(f"Failed {cur_test}: expected {expected_enu}, got {actual_enu}")

cur_test = "ned2enu zero test"
ned_point = [0, 0, 0]  # Single NED point with zero values
expected_enu = [0, 0, 0]  # Expected ENU point after conversion
actual_enu = Rotations.ned2enu(ned_point)

if not evaluateTest(cur_test, actual_enu == expected_enu):
    print(f"Failed {cur_test}: expected {expected_enu}, got {actual_enu}")
#%% getNewPoints():
print("Beginning testing of VehicleGeometry.getNewPoints()")

vehicle = VG.VehicleGeometry()

# Test 1: Identity test (no rotation, no translation)
cur_test = "getNewPoints identity test"
x, y, z = 0, 0, 0  # No translation
yaw, pitch, roll = 0, 0, 0  # No rotation
expected_points = Rotations.ned2enu(vehicle.vertices)
actual_points = vehicle.getNewPoints(x, y, z, yaw, pitch, roll)
if not evaluateTest(cur_test, actual_points == expected_points):
    print(f"Failed {cur_test}: expected {expected_enu}, got {actual_enu}")

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]