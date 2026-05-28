import math
import sys

import PyQt5.QtCore as QtCore
import PyQt5.QtWidgets as QtWidgets

import ece163.Display.baseInterface as baseInterface
import ece163.Containers.States as vehicleState
import ece163.Display.GridVariablePlotter
import ece163.Display.SliderWithValue

from ece163.Utilities.Joystick import Joystick
import ece163.Constants.JoystickConstants as JSC

stateNamesofInterest = ['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll']

positionRange = 10
updateTimedRate = 20
slidePeriod = 20

class testInterface(baseInterface.baseInterface):
	def __init__(self, parent=None):
		self.vehicleState = vehicleState.vehicleState()
		self.t = 0
		super().__init__(parent)
		self.stateGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(2, 3, [[x] for x in stateNamesofInterest], titles=stateNamesofInterest)

		self.outPutTabs.addTab(self.stateGrid, "States")
		self.outPutTabs.setCurrentIndex(2)
		self.stateUpdateDefList.append(self.updateStatePlots)

		self.inputGrid = QtWidgets.QGridLayout()
		self.inputLayout.addLayout(self.inputGrid)
		self.inputLayout.addStretch()
		self.inputSliders = list()

		
		self.joystick = Joystick()
		self.control_mode = 0
		self.disable_mode_toggle = 0

		for index, value in enumerate(stateNamesofInterest):
			if index > 2:
				row = 1
				minValue = -180
				maxValue = 180
			else:
				row = 0
				minValue = -positionRange
				maxValue = positionRange
			col = index % 3
			# print(row, col)
			if self.joystick.active:
				newSlider = ece163.Display.SliderWithValue.SliderWithValue(value, minValue, maxValue, onChangePointer=None)
			else:
				newSlider = ece163.Display.SliderWithValue.SliderWithValue(value, minValue, maxValue, onChangePointer=self.sliderChangeResponse)
			self.inputSliders.append(newSlider)
			self.inputGrid.addWidget(newSlider, row, col)

		self.stateUpdateDefList.append(self.updateSliderValues)


		#Start simulation and enable play button so that the controller input can work right away
		if self.joystick.active:
			self.playButton.setDisabled(False)
			self.PlaySimulation()
		else:
			self.playButton.setDisabled(True)

		self.showMaximized()
		# self.vehicleInstance.leavePlaneTrail = False

		####Simulation Update code###
		# # Updates the simulation when tab is being changed
		self.outPutTabs.currentChanged.connect(self.newTabClicked)
		self.outPutTabs.setCurrentIndex(0)
		# Default for all graphs to be turned off
		self.updatePlotsOn()
		self.updatePlotsOff()
		# Overwrite simulationTimedThread function with modified sliderChangeResponse
		self.simulationTimedThread.timeout.connect(self.sliderChangeResponse)



		return

	def updateStatePlots(self, newState):
		stateList = list()
		for key in self.numericStatesDict.keys():
			stateList.append([getattr(newState, key)])
		self.stateGrid.addNewAllData(stateList)
		return
	
	def updateSliderValues(self, newState):
		if self.joystick.active:
			for slider in self.inputSliders:
				if slider.name in ['yaw', 'pitch', 'roll']:
					slider.setSlider(math.degrees(getattr(newState,slider.name)))
				else:
					slider.setSlider(getattr(newState,slider.name))
		return

	def getVehicleState(self):
		return self.vehicleState

	def runUpdate(self):
		
		if self.joystick.active:
			controller_values = self.joystick.get_joystick_values()

			#Toggle control mode if button is pressed
			if controller_values.mode_button and not self.disable_mode_toggle:
				self.control_mode = not self.control_mode
				self.disable_mode_toggle = 1
			#Make sure it doesn't rapidly toggle until button is released
			elif not controller_values.mode_button and self.disable_mode_toggle:
				self.disable_mode_toggle = 0
			

			#Only move when trigger is pressed
			if controller_values.trigger:
				if self.control_mode:
					#Control translation
					self.vehicleState.pn = controller_values.control_axes.Elevator/JSC.MAX_THROW*JSC.CHAPTER2_MAX_TRANSLATION
					self.vehicleState.pe = controller_values.control_axes.Aileron/JSC.MAX_THROW*JSC.CHAPTER2_MAX_TRANSLATION
					self.vehicleState.pd = -controller_values.control_axes.Throttle*JSC.CHAPTER2_MAX_TRANSLATION*2 + JSC.CHAPTER2_MAX_TRANSLATION
				else:
					#Control rotation
					self.vehicleState.yaw = math.radians((controller_values.control_axes.Throttle*2.0 - 1.0) * JSC.CHAPTER2_MAX_ANGLE)
					self.vehicleState.pitch = math.radians((controller_values.control_axes.Elevator/JSC.MAX_THROW) * JSC.CHAPTER2_MAX_ANGLE) 
					self.vehicleState.roll = math.radians((controller_values.control_axes.Aileron/JSC.MAX_THROW) * JSC.CHAPTER2_MAX_ANGLE)

		return

	def sliderChangeResponse(self, newValue = 0.0, name= ""):
		self.updatePlotsOn()

		#Don't use this part if the controller 
		if not self.joystick.active:
			if name in ['yaw', 'pitch', 'roll']:
				setattr(self.vehicleState, name, math.radians(newValue))
			else:
				if name == 'z':
					newValue = -1*newValue
				setattr(self.vehicleState, name, newValue)
		self.runSimulation()
		self.updatePlotsOff()
		return

	def resetSimulationActions(self):
		self.vehicleState = vehicleState.vehicleState()
		self.stateGrid.clearDataPointsAll()
		for slider in self.inputSliders:
			slider.resetSlider()

		#### Simulation Update Code ####
		self.outPutTabs.setCurrentIndex(0)

		#If using a controller, start the simulation right away
		if self.joystick.active:
			self.PlaySimulation()
		else:
			self.playButton.setDisabled(True)


	#### Simulation Update Code ##########

	# Updates a simulation widget when new tab clicked
	def newTabClicked(self):
		self.updatePlotsOn()
		self.updatePlotsOff()
		return

	# toggles the state grid widget
	def togglestateGridPlot(self, toggleIn):
		self.stateGrid.setUpdatesEnabled(toggleIn)
		return

	# Turns on all simulation plots
	def updatePlotsOn(self):
		# print("Turning on plot update")
		self.togglestateGridPlot(True)
		return

	# Turns off all simulation plots
	def updatePlotsOff(self):
		# print("Turning off plot update")
		self.togglestateGridPlot(False)
		return
#######################

sys._excepthook = sys.excepthook

def my_exception_hook(exctype, value, tracevalue):
	# Print the error and traceback
	import traceback
	with open("LastCrash.txt", 'w') as f:
		traceback.print_exception(exctype, value, tracevalue, file=f)
	print(exctype, value, tracevalue)
	# Call the normal Exception hook after
	sys._excepthook(exctype, value, tracevalue)
	sys.exit(0)

# Set the exception hook to our wrapping function
sys.excepthook = my_exception_hook



qtApp = QtWidgets.QApplication(sys.argv)
ourWindow = testInterface()
ourWindow.show()
qtApp.exec()