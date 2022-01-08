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
import subsystems.intake
import networktables


class In(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot
		self.hasRequirement(self.robot.intake)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.intake] )

	# Methods
	def initialize(self):
		self.robot.intake.intake()

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.intake.stop()

	def interrupted(self):
		self.end()

class Out(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot
		self.hasRequirement(self.robot.intake)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.intake] )

	# Methods
	def initialize(self):
		self.robot.intake.outtake()

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.intake.stop()

	def interrupted(self):
		self.end()

class Up(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()
		self.robot = robot
		self.hasRequirement(self.robot.intake)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.intake] )

	# Methods
	def initialize(self):
		self.robot.intake.up()

	def isFinished(self):
		return True

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()

class Down(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()
		self.robot = robot
		self.hasRequirement(self.robot.intake)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.intake] )

	# Methods
	def initialize(self):
		self.robot.intake.down()

	def isFinished(self):
		return True

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()

class Stop(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot
		self.hasRequirement(self.robot.intake)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.intake] )

	# Methods
	def initialize(self):
		self.robot.intake.stop()

	def isFinished(self):
		return True

	def end(self, interrupted):
		self.robot.intake.stop()

	def interrupted(self):
		self.end()