"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020-2021
Code for robot "Krono-Z"
contact@team4096.org
"""

# This is to help vscode
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

# from wpilib.command import Command, CommandGroup, WaitCommand
from commands2 import Command

import const
import subsystems.turret
import networktables

from robot import Robot

### Degree Convention ###
# 0 degrees corresponds to the shooter facing AWAY from the intake
# Positive degree values go counterclockwise

class Run(Command):
	def __init__(self, robot: "Robot", speed_func):
		super().__init__()
		self.robot = robot
		self.speed_func = speed_func

		self.hasRequirement(self.robot.turret)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.turret] )

	# Methods
	def execute(self):
		self.robot.turret.run(self.speed_func())

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.turret.stop()

	def interrupted(self):
		self.end()

class Stop(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()
		self.robot = robot

		self.hasRequirement(self.robot.drive)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.turret] )

	# Methods
	def execute(self):
		self.robot.turret.stop()

	def isFinished(self):
		return True

	def end(self, interrupted):
		self.robot.turret.stop()

	def interrupted(self):
		self.end()


class Set_Turret_Angle(Command):
	def __init__(self, robot: "Robot", angle):
		'''
		initializes tank drive movement.
		:param robot: the robot object
		:param angle: angle to set servo to
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.turret)
		# self.setInterruptible(True)

		self.angle = angle

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.turret] )

	# Methods
	def execute(self):
		"""
		Get r and y values from our joystick axes and send them to subsystem
		"""
		# Finish this
		pass

	def isFinished(self):
		return False

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()



class Rotate_To_Angle_Limelight(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.turret)
		# self.setInterruptible(True)
		# self.setTimeout(5)	# For now... probably remove later

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.turret] )

	# Methods
	def initialize(self):
		self.robot.turret.rotate_pid.setSetpoint(0)

	def execute(self):
		output = self.robot.turret.rotate_pid.calculate(self.robot.limelight.get_angle_to_target())
		self.robot.turret.use_rotate_pid_output(output)

	def isFinished(self):
		# Eventually I think we want this to never finish. It's always tracking the
		# goal if it is in view of the limelight.

		on_target = abs(self.robot.turret.rotate_pid.getPositionError()) < 1.0
		# on_target = abs(self.robot.limelight.get_angle_to_target()) < 0.5
		timed_out = self.isTimedOut()

		if timed_out:
			print('Rotate To Angle Limelight Timed out!')
			return True
		if on_target:
			print('Rotate On Target! {0:.2f}'.format(self.robot.limelight.get_angle_to_target()))
			return True

		# keep going
		return False

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()