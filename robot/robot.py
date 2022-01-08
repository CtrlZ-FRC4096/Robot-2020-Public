#! python3


"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020-2021
Code for robot "Krono-Z"
contact@team4096.org
"""

DEBUG = True

# Import our files
import logging
import time
import sys

import wpilib
import commands2

import networktables

import const
import oi

from subsystems.drivetrain import Drivetrain
from subsystems.limelight import Limelight
from subsystems.turret import Turret
from subsystems.hood import Hood
from subsystems.shooter import Shooter
from subsystems.climber import Climber
from subsystems.indexer import Indexer
from subsystems.intake import Intake

from remote_shell import RemoteShell

import commands.autonomous

import commands.hood

import random


log = logging.getLogger('robot')
networktables.NetworkTables.initialize()

class Robot(commands2.TimedCommandRobot):
	"""
	Main robot class.

	This is the central object, holding instances of all the robot subsystem
	and sensor classes.

	It also contains the init & periodic methods for autonomous and
	teloperated modes, called during mode changes and repeatedly when those
	modes are active.

	The one instance of this class is also passed as an argument to the
	various other classes, so they have full access to all its properties.
	"""
	def robotInit(self):

		self.match_time = -1

		# NetworkTables
		self.nt_robot = networktables.NetworkTables.getTable('Robot')

		# Remote Shell
		if DEBUG:
			self.remote_shell = RemoteShell(self)
			print("Started Remote Shell")

		### Subsystems ###

		self.drive = Drivetrain(self)
		self.turret = Turret(self)
		self.hood = Hood(self)
		self.shooter = Shooter(self)
		self.limelight = Limelight(self)
		self.climber = Climber(self)
		self.indexer = Indexer(self)
		self.intake = Intake(self)

		self.subsystems = [
			self.drive,
			self.turret,
			self.hood,
			self.shooter,
			self.limelight,
			self.climber,
			self.indexer,
			self.intake,
		]

		### OTHER ###

		# Driverstation
		self.driverstation : wpilib.DriverStation = wpilib.DriverStation.getInstance()

		# Operator Input
		self.oi = oi.OI(self)

		## Scheduler ##
		self.scheduler = commands2.CommandScheduler.getInstance()

		## Autonomous ##
		# self.hood_distance_command = commands.hood.Set_Hood_Using_Distance(self)

		self.auto_choose = wpilib.SendableChooser()

		self.auto_choose.addOption( 'Do Nothing', commands.autonomous.Do_Nothing( self ) )
		self.auto_choose.addOption( 'Cross Baseline', commands.autonomous.Cross_Baseline( self ) )
		self.auto_choose.addOption( 'Behind_10_Foot', commands.autonomous.Behind_10_Foot( self ) )
		self.auto_choose.addOption( 'Front_10_Foot', commands.autonomous.Front_10_Foot( self ))
		self.auto_choose.addOption( 'Eleven Foot', commands.autonomous.Eleven_Foot(self))
		wpilib.SmartDashboard.putData('Auto Chooser', self.auto_choose)

		# Pressure Sensor
		self.pressure_sensor = wpilib.AnalogInput(const.AIN_PRESSURE_SENSOR)

		# Timer for pressure sensor's running average
		self._pressure_samples = []
		self._last_pressure_value = 0.0
		self.pressure_timer = wpilib.Timer()
		self.pressure_timer.start()
		self.pressure_timer_delay = 1.0		# times per second

		# Physics for Simulator
		# if wpilib.RobotBase.isSimulation():
		# 	import physics
		# 	self.physics = physics.PhysicsEngine()

		# 	self.last_tm = time.time()

		### LOGGING ###

		# Timers for NetworkTables update so we don't use too much bandwidth
		self.log_timer = wpilib.Timer()
		self.log_timer.start()
		self.log_timer_delay = 0.25		# 4 times/second

		# Disable LW telemetry before comp to improve loop dtimes
		# wpilib.LiveWindow.disableAllTelemetry()

		self.match_time = -1

		self.climber.undeploy()

		# self.selected_auto_simple = None
		self.selected_auto_mode = None

		#LEDs
		self.LEDS = wpilib.AddressableLED(1)
		self.LEDS.setLength(10)
		self.LEDS.start()


	def robotPeriodic(self):
		# Stuff for the wpilib simulator
		self.log()
		self.oi.log()

	### DISABLED ###

	def disabledInit(self):
		self.scheduler.cancelAll()

		self.limelight.set_startup_modes( )
		self.LEDS.setData([wpilib.AddressableLED.LEDData(0,0,0) for _ in range(10)])

		for s in self.subsystems:
			s.stop()


	def disabledPeriodic(self):
		pass
		self.limelight.set_led_mode(const.LIMELIGHT_LED_MODE_PIPELINE)
		# self.limelight.set_led_mode(const.LIMELIGHT_LED_MODE_OFF)


	### AUTONOMOUS ###

	def autonomousInit(self):
		# Get the driver-selected auto mode from SmartDashboard and start it
		# self.selected_auto_mode = self.auto_choose.getSelected()
		# self.selected_auto_mode = commands.autonomous.Run_Pathweaver_Path(self, "Ball_Auto2.wpilib.json")
		self.selected_auto_mode = commands.autonomous.Six_Ball(self)

		print( 'Running Auto mode: {0}'.format( self.selected_auto_mode ) )
		# self.selected_auto_mode.start( )

		# path_filename = self.limelight.get_galactic_search_path()
		# self.auto_command = commands.autonomous.Run_Pathweaver_Path(self, path_filename)
		self.scheduler.schedule(self.selected_auto_mode)

	def autonomousPeriodic(self):
		pass
		self.scheduler.run()


	### TELEOPERATED ###

	def teleopInit(self):
		self.drive.gyro.reset( )

		# Set Limelight LEDs to the mode specified in the pipeline configuration
		self.limelight.set_led_mode(const.LIMELIGHT_LED_MODE_PIPELINE)

		# Set default hood command to just stop, which clears out the
		# previous default command set for autonomous distance-adjusting
		hood_stop_command = commands.hood.Set_Hood_Speed(self, 0)
		self.hood.setDefaultCommand(hood_stop_command)

		# Removes any leftover commands from the scheduler
		self.scheduler.cancelAll()

		# Drive encoders
		self.drive.encoder_left.reset()
		self.drive.encoder_right.reset()


	def teleopPeriodic(self):

		# TEMPORARY!  Remove this once kinks are worked out.
		# self.hood.setDefaultCommand(self.hood_distance_command)
		# End temporary

		self.scheduler.run()


	### MISC ###

	def get_pressure(self):
		"""
		Calculate a running average of pressure values.  The sensor seems to jitter its values
		a lot, so this smoothes it out and make the Shuffleboard display more readable.
		"""
		voltage_pressure = self.pressure_sensor.getVoltage()
		new_value = (250 * voltage_pressure / 5) - 25

		self._pressure_samples.append(new_value)

		if not self.pressure_timer.hasPeriodPassed(self.pressure_timer_delay):
			return self._last_pressure_value

		# Calculate new running average
		new_avg = sum(self._pressure_samples) / len(self._pressure_samples)

		self._pressure_samples = [ ]
		self._last_pressure_value = new_avg

		return new_avg


	def log(self):
		"""
		Logs some info to shuffleboard, and standard output
		"""
		if not self.log_timer.advanceIfElapsed(self.log_timer_delay):
			return

		self.nt_robot.putString('Pressure', '{0:.2f}'.format(self.get_pressure()))

		for s in self.subsystems:
			s.log()

		self.match_time = self.driverstation.getMatchTime()

### MAIN ###

if __name__ == "__main__":
	wpilib.run(Robot)
