"""
widget which handles the creation of linear models and determining the gains
"""

import PyQt5.QtWidgets as QtWidgets
from .SliderWithValue import SliderWithValue
from ..Constants import VehiclePhysicalConstants
from ..Constants import JoystickConstants as JSC
from ..Containers.Controls import referenceCommands
import math

# Airspeed, Altitude, Course
class ReferenceControlWidget(QtWidgets.QWidget):
	def __init__(self, callBackOnChange=None, parent=None):
		super().__init__()

		self.callBack = callBackOnChange
		self.usedLayout = QtWidgets.QVBoxLayout()
		self.setLayout(self.usedLayout)
		self.currentReference = referenceCommands()

		self.airSpeedInput = SliderWithValue("Airspeed", JSC.CHAPTER6_MIN_AIRSPEED, JSC.CHAPTER6_MAX_AIRSPEED, VehiclePhysicalConstants.InitialSpeed, self.referenceChanged)
		self.usedLayout.addWidget(self.airSpeedInput)

		self.altitudeInput = SliderWithValue('Altitude', JSC.CHAPTER6_MIN_ALTITUDE, JSC.CHAPTER6_MAX_ALTITUDE, -VehiclePhysicalConstants.InitialDownPosition, self.referenceChanged)
		self.usedLayout.addWidget(self.altitudeInput)

		self.courseInput = SliderWithValue('Course', JSC.CHAPTER6_MIN_COURSE, JSC.CHAPTER6_MAX_COURSE, VehiclePhysicalConstants.InitialYawAngle, self.referenceChanged)
		self.usedLayout.addWidget(self.courseInput)

		self.buildCurrentReferences()
		self.usedLayout.addStretch()
		return

	def referenceChanged(self, unusedValue, unusedName):
		self.buildCurrentReferences()
		return

	def buildCurrentReferences(self):
		try:
			courseCommand = math.radians(self.courseInput.curValue)
			altitudeCommand = self.altitudeInput.curValue
			airspeedCommand = self.airSpeedInput.curValue
			self.currentReference = referenceCommands(courseCommand, altitudeCommand, airspeedCommand)
		except AttributeError:  # this stops weird loading bug due to referencing it before it exists
			pass

	def setSliders(self, ref):
		self.airSpeedInput.setSlider(ref.commandedAirspeed)
		self.altitudeInput.setSlider(ref.commandedAltitude)
		self.courseInput.setSlider(math.degrees(ref.commandedCourse))
