import math
import sys

import PyQt5.QtWidgets as QtWidgets

import ece163.Display.baseInterface as baseInterface
import ece163.Containers.Inputs as Inputs
import ece163.Display.GridVariablePlotter
import ece163.Display.SliderWithValue
import ece163.Simulation.Chapter3Simulate
import ece163.Display.DataExport

from ece163.Utilities.Joystick import Joystick
import ece163.Constants.JoystickConstants as JSC

stateNamesofInterest = ['pn', 'pe', 'pd', 'yaw', 'pitch', 'roll', 'u', 'v', 'w', 'p', 'q', 'r']
systemInputs = [('Fx', -10, 10, 0),
				('Fy', -10, 10, 0),
				('Fz', -10, 10, 0),
				('Mx', -0.1, 0.1, 0),
				('My', -0.1, 0.1, 0),
				('Mz', -0.1, 0.1, 0)]

positionRange = 200

class Chapter3(baseInterface.baseInterface):
	def __init__(self, parent=None):
		# self.vehicleState = vehicleState.vehicleState()
		self.simulateInstance = ece163.Simulation.Chapter3Simulate.Chapter3Simulate()
		super().__init__(parent)
		self.setWindowTitle("ECE163 Chapter 3")
		self.stateGrid = ece163.Display.GridVariablePlotter.GridVariablePlotter(4, 3, [[x] for x in stateNamesofInterest], titles=stateNamesofInterest)

		self.outPutTabs.addTab(self.stateGrid, "States")
		self.outPutTabs.setCurrentIndex(2)
		self.stateUpdateDefList.append(self.updateStatePlots)

		self.exportWidget = ece163.Display.DataExport.DataExport(self.simulateInstance,'Chapter3')
		self.outPutTabs.addTab(self.exportWidget, "Export Data")

		self.inputGrid = QtWidgets.QGridLayout()
		self.inputLayout.addLayout(self.inputGrid)

		resetSlidersButton = QtWidgets.QPushButton("Reset Sliders")
		self.inputLayout.addWidget(resetSlidersButton)
		resetSlidersButton.clicked.connect(self.resetSliders)

		self.inputLayout.addStretch()
		self.inputSliders = list()

		for row in range(2):
			for col in range(3):
				index = col+row*3
				name, minValue, maxValue, startValue = systemInputs[index]
				newSlider = ece163.Display.SliderWithValue.SliderWithValue(name, minValue, maxValue, startValue)
				self.inputSliders.append(newSlider)
				self.inputGrid.addWidget(newSlider, row, col)

		# self.playButton.setDisabled(True)
		self.showMaximized()

		#Initialize controller plus some variables to track control state switching
		self.joystick = Joystick()
		self.control_mode = 0
		self.disable_mode_toggle = 0


		####Simulation Update code###
		# # Updates the simulation when tab is being changed
		self.outPutTabs.currentChanged.connect(self.newTabClicked)
		self.outPutTabs.setCurrentIndex(0)
		self.plotWidgets = [self.stateGrid]
		# Default for all graphs to be turned off
		self.updatePlotsOn()
		self.updatePlotsOff()
		# Overwrite simulationTimedThread function with modified sliderChangeResponse
		self.simulationTimedThread.timeout.connect(self.UpdateSimulationPlots)

		return 

	def resetSliders(self):
		for slider in self.inputSliders:
			slider.resetSlider()
		return

	def updateStatePlots(self, newState):
		stateList = list()
		for key in stateNamesofInterest:
			newVal = getattr(newState, key)
			if key in ['yaw', 'pitch', 'roll', 'p', 'q', 'r']:
				newVal = math.degrees(newVal)
			stateList.append([newVal])

		self.stateGrid.addNewAllData(stateList, [self.simulateInstance.time]*len(stateNamesofInterest))
		return

	def getVehicleState(self):
		return self.simulateInstance.underlyingModel.getVehicleState()

	def runUpdate(self):
		forceInputs = Inputs.forcesMoments()


		if self.joystick.active:
			joystick_values = self.joystick.get_joystick_values()

			#Toggle control mode if button is pressed
			if joystick_values.mode_button and not self.disable_mode_toggle:
				self.control_mode = not self.control_mode
				self.disable_mode_toggle = 1
			#Make sure it doesn't rapidly toggle until button is released
			elif not joystick_values.mode_button and self.disable_mode_toggle:
				self.disable_mode_toggle = 0
			

			#Only update values when trigger is pressed
			if joystick_values.trigger:
				if self.control_mode:
					#Control forces
					forceInputs.Fx = joystick_values.control_axes.Elevator/JSC.MAX_THROW*JSC.CHAPTER3_MAX_FORCE
					forceInputs.Fy = joystick_values.control_axes.Aileron/JSC.MAX_THROW*JSC.CHAPTER3_MAX_FORCE
					forceInputs.Fz = -(joystick_values.control_axes.Throttle*2.0 - 1.0)*JSC.CHAPTER3_MAX_FORCE
				else:
					#Control moments
					forceInputs.Mz = -(joystick_values.control_axes.Throttle*2.0 - 1.0) * JSC.CHAPTER3_MAX_MOMENT
					forceInputs.My = -(joystick_values.control_axes.Elevator/JSC.MAX_THROW) * JSC.CHAPTER3_MAX_MOMENT
					forceInputs.Mx = (joystick_values.control_axes.Aileron/JSC.MAX_THROW) * JSC.CHAPTER3_MAX_MOMENT

				for slider in self.inputSliders:
					slider.setSlider(getattr(forceInputs,slider.name))
		else:
			#Use sliders if controller not active
			for control in self.inputSliders:
				setattr(forceInputs, control.name, control.curValue)


		self.simulateInstance.takeStep(forceInputs)

		return

	def resetSimulationActions(self):
		self.simulateInstance.reset()
		self.stateGrid.clearDataPointsAll()

		#### Simulation Update Code ##########

	# Updates a simulation widget when new tab clicked
	def UpdateSimulationPlots(self):

		currentWidget = self.outPutTabs.currentWidget()
		# Ensure that that the timer is only enabled for states, sensors, and control response widgets
		if (currentWidget in self.plotWidgets):
			self.runUpdate()
			self.updatePlotsOn()
			self.updatePlotsOff()
		return
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


sys._excepthook = sys.excepthook

def my_exception_hook(exctype, value, tracevalue):
	# Print the error and traceback
	import traceback
	with open("LastCrash.txt", 'w') as f:
		traceback.print_exception(exctype, value, tracevalue, file=f)
		# traceback.print_tb(tracevalue, file=f)
	print(exctype, value, tracevalue)
	# Call the normal Exception hook after
	sys._excepthook(exctype, value, tracevalue)
	sys.exit(0)

# Set the exception hook to our wrapping function
sys.excepthook = my_exception_hook


qtApp = QtWidgets.QApplication(sys.argv)
ourWindow = Chapter3()
ourWindow.show()
qtApp.exec()