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
import subsystems.indexer
import networktables

class Feed(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.indexer)
		self.hasRequirement(self.robot.intake)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.indexer] )

	# Methods
	def initialize(self):
		self.robot.indexer.feed()
		self.robot.intake.run(0.1)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.indexer.stop()
		self.robot.intake.stop()

	def interrupted(self):
		self.end()

class AutoFeed(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot
		self.hasRequirement(self.robot.indexer)
		self.hasRequirement(self.robot.intake)
		# self.setInterruptible(True)
		# self.setTimeout(10)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.indexer] )

	def isTimedOut(self):
		return False

	# Methods
	def execute(self):
		self.robot.indexer.intake()
		self.robot.indexer.run_bottom(0.85)
		self.robot.intake.run(1.0)
		self.robot.intake.down


	def isFinished(self):
		if self.isTimedOut():
			return True
		return False

	def end(self, interrupted):
		self.robot.indexer.stop()
		self.robot.intake.stop()

	def interrupted(self):
		self.end()

class In(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot
		self.hasRequirement(self.robot.indexer)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.indexer] )

	# Methods
	def initialize(self):
		self.robot.indexer.intake()

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.indexer.stop()

	def interrupted(self):
		self.end()

class Out(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot
		self.hasRequirement(self.robot.indexer)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.indexer] )

	# Methods
	def initialize(self):
		self.robot.indexer.outtake()

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.indexer.stop()

	def interrupted(self):
		self.end()

class Shuffle(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot
		self.hasRequirement(self.robot.indexer)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.indexer] )

	# Methods
	def initialize(self):
		self.robot.indexer.shuffle()

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.indexer.stop()

	def interrupted(self):
		self.end()

class Stop(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot
		self.hasRequirement(self.robot.indexer)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.indexer] )

	# Methods
	def initialize(self):
		self.robot.indexer.stop()

	def isFinished(self):
		return True

	def end(self, interrupted):
		self.robot.indexer.stop()

	def interrupted(self):
		self.end()
