from . import Simulate
from ..Controls import VehicleClosedLoopControl
from ..Containers.Controls import referenceCommands
from ece163.Controls.VehicleTrim import VehicleTrim
from ..Constants import VehiclePhysicalConstants
from ..Sensors import SensorsModel

class Chapter8Simulate(Simulate.Simulate):
	def __init__(self):
		super().__init__()
		self.inputNames.extend(['commandedCourse', 'commandedAltitude', 'commandedAirspeed'])
		self.underlyingModel = VehicleClosedLoopControl.VehicleClosedLoopControl(rudderControlSource='YAW', useSensors=True, useEstimator=True)
		self.sensorModel = self.underlyingModel.getSensorsModel()
		self.vehicleEstimator = self.underlyingModel.getVehicleEstimator()

		# self.variableList.append((self.underlyingModel.getForcesMoments, 'ForceMoments',
		# 							['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']))

		self.variableList.append((self.underlyingModel.getVehicleState, 'state',
								['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r', 'Va', 'alpha', 'beta', 'chi']))
		# self.dT = 1/50
		self.variableList.append((self.sensorModel.getSensorsNoisy, 'sensors_noisy',
								['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z', 'mag_x', 'mag_y', 'mag_z', 'baro', 'pitot', 'gps_n', 'gps_e', 'gps_alt', 'gps_sog', 'gps_cog']))
		self.variableList.append((self.sensorModel.getSensorsTrue, 'sensors_true',
								['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z', 'mag_x', 'mag_y', 'mag_z', 'baro', 'pitot', 'gps_n', 'gps_e', 'gps_alt', 'gps_sog', 'gps_cog']))
		self.variableList.append((self.vehicleEstimator.getEstimatedState, 'estimated_state',
								['yaw', 'pitch', 'roll','p', 'q', 'r', 'Va','chi']))
		self.referenceInput = referenceCommands()

	def getVehicleState(self):
		return self.underlyingModel.getVehicleState()

	def takeStep(self, referenceInput=None):
		self.time += VehiclePhysicalConstants.dT
		if referenceInput is None:
			referenceInput = self.referenceInput
		self.underlyingModel.update(referenceInput)
		self.recordData([referenceInput.commandedCourse, referenceInput.commandedAltitude, referenceInput.commandedAirspeed])
		return

	def reset(self):
		self.time = 0
		self.underlyingModel.reset()
		self.takenData.clear()