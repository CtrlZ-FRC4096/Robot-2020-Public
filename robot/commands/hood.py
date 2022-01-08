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
from commands2 import CommandBase

import const

class Set_Hood_Position(CommandBase):
	def __init__(self, robot: "Robot", position):
		'''
		initializes hood
		:param robot: the robot object
		:param angle: angle to set servo to
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.hood)

		self.position = position

	# Overrides
	def getRequirements(self):
		return set( [self.robot.hood] )

	def initialize(self):
		self.robot.hood.rotate_pid.reset()

	def execute(self):
		x = self.robot.hood.rotate_pid.calculate(self.robot.hood.pot.get(), self.position)
		self.robot.hood.servo1.set(x)


	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.hood.servo1.set(0)
		self.robot.hood.rotate_pid.reset()

	def interrupted(self):
		self.end()


class Set_Hood_Speed(CommandBase):
	def __init__(self, robot: "Robot", speed):
		'''
		initializes hood
		:param robot: the robot object
		:param speed: speed to set servo to
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.hood)

		self.speed = speed

	# Overrides
	def getRequirements(self):
		return set( [self.robot.hood] )

	def execute(self):
		"""
		"""
		self.robot.hood.servo1.set(self.speed)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.hood.servo1.set(0)

	def interrupted(self):
		self.end()


class Find_Hood_Angle(CommandBase):
	def __init__(self, robot: "Robot"):
		'''
		initializes hood
		:param robot: the robot object
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.hood)

	# Overrides
	def getRequirements(self):
		return set( [self.robot.hood] )

	def execute(self):
		"""
		"""
		if not self.robot.limelight.is_target_visible():
			return

		# Dynamically adjusts hood
		distance_to_target = self.robot.limelight.get_distance_to_target()
		hood_angle = const.HOOD_A * distance_to_target ** 2 + const.HOOD_B * distance_to_target + const.HOOD_C
		adjusted_hood_angle = round(hood_angle*2+.5)/2
		return adjusted_hood_angle


	def isFinished(self):
		return False

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()


class Set_Hood_Using_Distance(CommandBase):
	def __init__(self, robot: "Robot"):
		'''
		initializes hood
		:param robot: the robot object
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.hood)

	# Overrides
	def getRequirements(self):
		return set( [self.robot.hood] )

	def execute(self):
		"""
		"""
		if not self.robot.limelight.is_target_visible():
			return

		# Dynamically adjusts hood
		distance_to_target = self.robot.limelight.get_distance_to_target()

		if distance_to_target > const.HOOD_MAX_DISTANCE:
			hood_angle = const.HOOD_MAX_DISTANCE_ANGLE
		elif distance_to_target < const.HOOD_MIN_DISTANCE:
			hood_angle = const.HOOD_MIN_DISTANCE_ANGLE
		else:
			hood_angle = const.HOOD_A * distance_to_target ** 2 + const.HOOD_B * distance_to_target + const.HOOD_C

		# Round to nearest degree
		hood_angle = int(hood_angle + 0.5)

		hood_cmd = Set_Hood_Position(self.robot, hood_angle)
		self.robot.scheduler.schedule(hood_cmd)

		self.robot.nt_robot.putNumber('Hood Angle', hood_angle)


	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.hood.servo1.set(0)

	def interrupted(self):
		self.end()