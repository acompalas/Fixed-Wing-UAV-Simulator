"""
File contains the classes for controls primitives, gains, and tuning knobs for tuning controls models.
"""

import math
import enum

from ..Constants import VehiclePhysicalConstants as VPC

testingAbs_tol = 1e-4

class AltitudeStates(enum.Enum):
	"""
	class AltitudeStates(enum.Enum):
	Enumeration class for the altitude hold state machine. Defines three states that we will be using to reset the PI integrators
	when switching between different altitudes.
	"""
	CLIMBING = enum.auto()
	HOLDING = enum.auto()
	DESCENDING = enum.auto()

class referenceCommands():
	def __init__(self, courseCommand=VPC.InitialYawAngle, altitudeCommand=-VPC.InitialDownPosition, airspeedCommand=VPC.InitialSpeed):
		"""
		def __init__(self, courseCommand=VPC.InitialYawAngle, altitudeCommand=-VPC.InitialDownPosition, airspeedCommand=VPC.InitialSpeed):
		Class to hold the commanded inputs for closed loop control.

		:param courseCommand: commanded course over ground [rad], measured + from Inertial North
		:param altitudeCommand: commanded height [m]
		:param airspeedCommand: commanded airspeed [m/s]
		:return: none
		"""
		self.commandedCourse = courseCommand
		self.commandedAltitude = altitudeCommand
		self.commandedAirspeed = airspeedCommand
		self.commandedRoll = 0.0	# These will be set by the control loops internally to the successive loop closure
		self.commandedPitch = 0.0	# These will be set by the control loops internally to the successive loop closure
		return

	def __str__(self):
		return "referenceCommands:(course={}, altitude={}, " \
			   "airspeed={}, roll={}, pitch={}".format(math.degrees(self.commandedCourse), self.commandedAltitude,
													   self.commandedAirspeed, math.degrees(self.commandedRoll),
													   math.degrees(self.commandedPitch))

	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all([math.isclose(getattr(self, member), getattr(other, member), abs_tol=testingAbs_tol) for member in ['commandedCourse',
																								   'commandedAltitude',
																								   'commandedAirspeed',
																								   'commandedRoll',
																								   'commandedPitch']]):
				return False
			else:
				return True
		else:
			return NotImplemented


class controlGains():
	def __init__(self, kp_roll = 0.0, kd_roll = 0.0, ki_roll = 0.0, kp_sideslip = 0.0, ki_sideslip = 0.0, kp_course = 0.0, ki_course = 0.0, kp_pitch = 0.0, kd_pitch = 0.0, kp_altitude = 0.0, ki_altitude = 0.0, kp_SpeedfromThrottle = 0.0, ki_SpeedfromThrottle = 0.0, kp_SpeedfromElevator = 0.0, ki_SpeedfromElevator = 0.0):
		"""
		Class to hold the control gains for both lateral and longitudinal autopilots in the successive loop closure method
		described in Beard Chapter 6.

		:param kp_roll: roll proportional gain
		:param kd_roll: roll derivative gain
		:param ki_roll: roll integral gain
		:param kp_sideslip: sideslip proportional gain
		:param ki_sideslip: sideslip integral gain
		:param kp_course: course proportional gain
		:param ki_course: course integral gain
		:param kp_pitch: pitch proportional gain
		:param kd_pitch: pitch derivative gain
		:param kp_altitude: altitude proportional gain
		:param ki_altitude: altitude integral gain
		:param kp_SpeedfromThrottle: airspeed (from throttle) proportional gain
		:param ki_SpeedfromThrottle: airspeed (from throttle) integral gain
		:param kp_SpeedfromElevator: airspeed (from elevator) proportional gain
		:param ki_SpeedfromElevator: airspeed (from elevator) integral gain

		:return: none
		"""
		# Lateral Gains
		self.kp_roll = kp_roll
		self.kd_roll = kd_roll
		self.ki_roll = ki_roll
		self.kp_sideslip = kp_sideslip
		self.ki_sideslip = ki_sideslip
		self.kp_course = kp_course
		self.ki_course = ki_course
		# Longitudinal Gains
		self.kp_pitch = kp_pitch
		self.kd_pitch = kd_pitch
		self.kp_altitude = kp_altitude
		self.ki_altitude = ki_altitude
		self.kp_SpeedfromThrottle = kp_SpeedfromThrottle
		self.ki_SpeedfromThrottle = ki_SpeedfromThrottle
		self.kp_SpeedfromElevator = kp_SpeedfromElevator
		self.ki_SpeedfromElevator = ki_SpeedfromElevator
		return

	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all([math.isclose(getattr(self, member), getattr(other, member), abs_tol=testingAbs_tol)
						for member in ['kp_roll', 'kd_roll', 'ki_roll', 'kp_sideslip', 'ki_sideslip', 'kp_course',
									   'ki_course', 'kp_pitch', 'kd_pitch', 'kp_altitude', 'ki_altitude',
									   'kp_SpeedfromThrottle', 'ki_SpeedfromThrottle', 'kp_SpeedfromElevator',
									   'ki_SpeedfromElevator']]):
				return False
			else:
				return True
		else:
			return NotImplemented

	def __repr__(self):
		return "{0.__name__}(kp_roll={1.kp_roll}, kd_roll={1.kd_roll}, ki_roll={1.ki_roll}, kp_sideslip={1.kp_sideslip}, " \
			   "ki_sideslip={1.ki_sideslip}, kp_course={1.kp_course}, ki_course={1.ki_course}, kp_pitch={1.kp_pitch}, " \
			   "kd_pitch={1.kd_pitch}, kp_altitude={1.kp_altitude}, ki_altitude={1.ki_altitude}, kp_SpeedfromThrottle={1.kp_SpeedfromThrottle}, " \
			   "ki_SpeedfromThrottle={1.ki_SpeedfromThrottle}, kp_SpeedfromElevator={1.kp_SpeedfromElevator}, " \
			   "ki_SpeedfromElevator={1.ki_SpeedfromElevator})".format(type(self), self)

class VehicleEstimatorGains:
	def __init__(self, Kp_acc=0.0, Ki_acc=0.0, Kp_mag=0.0, Ki_mag=0.0, Kp_h = 0.0, Ki_h = 0.0, Kp_h_gps = 0.0, Ki_h_gps = 0.0, lowPassCutoff_h=0.0, Kp_Va=0.0, Ki_Va = 0.0, Kp_chi=0.0, Ki_chi=0.0):
		"""
		Class to hold the gains for the estimators used to estimate the vehicle state from the noisy sensor measurements.
		Currently using complementary filter for attitude estimation. Will add more as we complete the lab writup.

		:param Kp_acc: Attitude Estimator accelerometer proportional gain
		:param Ki_acc: Attitude Estimator accelerometer integral gain
		:param Kp_mag: Attitude Estimator magnetometer proportional gain
		:param Ki_mag: Attitude Estimator magnetometer integral gain
		:param Kp_h: Altitude Estimator proportional gain
		:param Ki_h: Altitude Estimator integral gain
		:param Kp_h_gps: Altitude Estimator proportional gain for GPS update
		:param Ki_h_gps: Altitude Estimator integral gain for GPS update
		:param lowPassCutoff_h: Altitude Estimator baro low pass filter cutoff frequency
		:param Kp_Va: Airspeed Estimator proportional gain
		:param Ki_Va: Airspeed Estimator integral gain
		:param Kp_chi: Course Estimator proportional gain
		:param Ki_chi: Course Estimator integral gain
		:return: None
		"""

		self.Kp_acc = Kp_acc
		self.Ki_acc = Ki_acc
		self.Kp_mag = Kp_mag
		self.Ki_mag = Ki_mag
		self.Kp_h = Kp_h
		self.Ki_h = Ki_h
		self.Kp_h_gps = Kp_h_gps
		self.Ki_h_gps = Ki_h_gps
		self.lowPassCutoff_h = lowPassCutoff_h
		self.Kp_Va = Kp_Va
		self.Ki_Va = Ki_Va
		self.Kp_chi = Kp_chi
		self.Ki_chi = Ki_chi
		return
	
	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all([math.isclose(getattr(self, member), getattr(other, member), abs_tol=testingAbs_tol)
						for member in ['Kp_acc', 'Ki_acc', 'Kp_mag', 'Ki_mag', 'Kp_h', 'Ki_h', 'Kp_h_gps', 'Ki_h_gps', 'lowPassCutoff_h', 'Kp_Va', 'Ki_Va', 'Kp_chi', 'Ki_chi']]):
				return False
			else:
				return True
		else:
			return NotImplemented
		
	def __repr__(self):
		return "{0.__name__}(Kp_acc={1.Kp_acc}, Ki_acc={1.Ki_acc}, Kp_mag={1.Kp_mag}, Ki_mag={1.Ki_mag}, Kp_h={1.Kp_h}, Ki_h={1.Ki_h}, Kp_h_gps={1.Kp_h_gps}, Ki_h_gps{1.Ki_h_gps}, lowPassCutoff_h={1.lowPassCutoff_h}, Kp_Va={1.Kp_Va}, Ki_Va={1.Ki_Va}, Kp_chi={1.Kp_chi}, Ki_chi={1.Ki_chi})".format(type(self), self)
	

class controlTuning():
	def __init__(self, Wn_roll = 0.0, Zeta_roll = 0.0, Wn_course = 0.0, Zeta_course = 0.0, Wn_sideslip = 0.0, Zeta_sideslip = 0.0, Wn_pitch = 0.0, Zeta_pitch = 0.0, Wn_altitude = 0.0	, Zeta_altitude = 0.0, Wn_SpeedfromThrottle = 0.0, Zeta_SpeedfromThrottle = 0.0, Wn_SpeedfromElevator = 0.0, Zeta_SpeedfromElevator = 0.0):
		"""
		Class to hold the tuning knobs for both lateral and longitudinal autopilots in the successive loop closure method
		described in Beard Chapter 6. Note that in the successive loop closure methodology, the gains are determined for
		the inner-most loop and the bandwidth separation is used to ensure that the outer loops do not interfere with the
		inner loops. Typical 5-10 ratios at a minimum between natural frequency of the loops.

		:return: none
		"""
		# tuning knobs for lateral control (ignoring Ki_phi)
		self.Wn_roll  = Wn_roll 
		self.Zeta_roll  = Zeta_roll 
		self.Wn_course  = Wn_course 	# Wn_roll should be 5-10x larger
		self.Zeta_course  = Zeta_course 
		self.Wn_sideslip  = Wn_sideslip 
		self.Zeta_sideslip  = Zeta_sideslip 
		#tuning knobs for longitudinal control
		self.Wn_pitch  = Wn_pitch 
		self.Zeta_pitch  = Zeta_pitch 
		self.Wn_altitude  = Wn_altitude 	# Wn_pitch should be 5-10x larger
		self.Zeta_altitude  = Zeta_altitude 
		self.Wn_SpeedfromThrottle  = Wn_SpeedfromThrottle 
		self.Zeta_SpeedfromThrottle  = Zeta_SpeedfromThrottle 
		self.Wn_SpeedfromElevator  = Wn_SpeedfromElevator 
		self.Zeta_SpeedfromElevator  = Zeta_SpeedfromElevator 
		return

	def __str__(self):
		return "{0.__name__}(Wn_roll={1.Wn_roll}, Zeta_roll={1.Zeta_roll}, Wn_course={1.Wn_course}, Zeta_course={1.Zeta_course}, " \
			   "Wn_sideslip={1.Wn_sideslip}, Zeta_sideslip={1.Zeta_sideslip}, Wn_pitch={1.Wn_pitch}, Zeta_pitch={1.Zeta_pitch}, " \
			   "Wn_altitude={1.Wn_altitude}, Zeta_altitude={1.Zeta_altitude}, Wn_SpeedfromThrottle={1.Wn_SpeedfromThrottle}, " \
			   "Zeta_SpeedfromThrottle={1.Zeta_SpeedfromThrottle}, Wn_SpeedfromElevator={1.Wn_SpeedfromElevator}, " \
			   "Zeta_SpeedfromElevator={1.Zeta_SpeedfromElevator})".format(type(self), self)



	def __eq__(self, other):
		if isinstance(other, type(self)):
			if not all([math.isclose(getattr(self, member), getattr(other, member), abs_tol=testingAbs_tol)
						for member in ['Wn_roll', 'Zeta_roll', 'Wn_course', 'Zeta_course', 'Wn_sideslip', 'Zeta_sideslip',
									   'Wn_pitch', 'Zeta_pitch', 'Wn_altitude', 'Zeta_altitude', 'Wn_SpeedfromThrottle',
									   'Zeta_SpeedfromThrottle', 'Wn_SpeedfromElevator', 'Zeta_SpeedfromElevator']]):
				return False
			else:
				return True
		else:
			return NotImplemented


