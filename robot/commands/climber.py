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
import subsystems.climber
import networktables

class Deploy(CommandBase):
	def __init__(self, robot: "Robot"):
		'''
		Explain what this command does here
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.climber)

	# Overrides
	def getRequirements(self):
		return set( [self.robot.climber] )

	def execute(self):
		self.robot.climber.deploy()

	def isFinished(self):
		return True

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()

class Unsafe_Undeploy(CommandBase):
	def __init__(self, robot: "Robot"):
		'''
		Explain what this command does here
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.climber)

	# Overrides
	def getRequirements(self):
		return set( [self.robot.climber] )

	def execute(self):
		self.robot.climber.undeploy()

	def isFinished(self):
		return True

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()

class Unsafe_Wind(CommandBase):
	def __init__(self, robot: "Robot"):
		'''
		Explain what this command does here
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.climber)

	# Overrides
	def getRequirements(self):
		return set( [self.robot.climber] )

	def initialize(self):
		self.robot.climber.wind()

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.climber.stop()
		pass

	def interrupted(self):
		self.end()

class Unsafe_Unwind(CommandBase):
	def __init__(self, robot: "Robot"):
		'''
		Explain what this command does here
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.climber)

	# Overrides
	def getRequirements(self):
		return set( [self.robot.climber] )

	def initialize(self):
		self.robot.climber.unwind()

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.climber.stop()
		pass

	def interrupted(self):
		self.end()

class Set_Brake(CommandBase):
	def __init__(self, robot, state):
		'''
		Explain what this command does here
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.climber)

		self.state = state

	# Overrides
	def getRequirements(self):
		return set( [self.robot.climber] )

	def execute(self):
		self.robot.climber.set_brake(self.state)

	def isFinished(self):
		return True

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()


Safe_Deploy = Deploy

class Safe_Undeploy(CommandBase):
	def __init__(self, robot: "Robot"):
		'''
		Explain what this command does here
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.climber)

	# Overrides
	def getRequirements(self):
		return set( [self.robot.climber] )

	def execute(self):
		if self.robot.climber.bottom_limit_switch.get():
			self.robot.climber.undeploy()

	def isFinished(self):
		return True

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()


# Safe_Wind() unwindes the climber string, whoops
class Safe_Wind(CommandBase):
	def __init__(self, robot: "Robot", speed=None):
		'''
		Explain what this command does here
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.climber)

		self.speed = speed
		self.is_callable = callable(speed)

	# Overrides
	def getRequirements(self):
		return set( [self.robot.climber] )

	def initialize(self):
		# if not self.isFinished():
		self.robot.climber.set_brake(False)
		# print("Safe_Wind.initialize(): RELeased brake")
		# else:
		#	print("Safe_Wind.initialize(): IGNORED brake")

	def execute(self):
		# print("Safe_Wind.execute() executing")
		if not self.isFinished():
			if self.is_callable:
				# print("Safe_Wind.execute() winding speed = {}\n".format(self.speed()))
				self.robot.climber.wind(self.speed())
			else:
				# print("Safe_Wind.execute() winding")
				self.robot.climber.wind()

	def isFinished(self):
		x = self.robot.climber.top_limit_switch.get()
		y = not self.robot.climber.is_deployed
		# print("Safe_Unwind.isFinished: top_limit_switch: {}  not climer.is_deployed: {}\n".format(x, y))
		return self.robot.climber.top_limit_switch.get() or not self.robot.climber.is_deployed

	def end(self, interrupted):
		self.robot.climber.stop()
		self.robot.climber.set_brake(True)
		# print("Safe_Wind.end():   SET brake")

	def interrupted(self):
		self.end()


class Safe_Unwind(CommandBase):
	def __init__(self, robot: "Robot", speed=None):
		'''
		Explain what this command does here
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.climber)

		self.speed = speed
		self.is_callable = callable(speed)

	# Overrides
	def getRequirements(self):
		return set( [self.robot.climber] )

	def initialize(self):
		# if not self.isFinished():
		self.robot.climber.set_brake(False)
		# print("Safe_Unwind.initialize: RELease brake")
		# else:
		#	print("Safe_Unwind.initialize: IGNORED RELease--NOT FINISHED")

	def execute(self):
		# print("Safe_Unwind.execute() executing")
		if not self.isFinished():
			if self.is_callable:
				self.robot.climber.unwind(self.speed())
			else:
				self.robot.climber.unwind()

	def isFinished(self):
		x = self.robot.climber.bottom_limit_switch.get()
		y = not self.robot.climber.is_deployed
		# print("Safe_Unwind.isFinished: bottom_limit_switch: {}  not climer.is_deployed: {}\n".format(x, y))
		return self.robot.climber.bottom_limit_switch.get() or not self.robot.climber.is_deployed

	def end(self, interrupted):
		self.robot.climber.stop()
		self.robot.climber.set_brake(True)
		# print("Safe_Unwind.end:        SET brake ")

	def interrupted(self):
		self.end()


class Stop(CommandBase):
	def __init__(self, robot: "Robot"):
		'''
		Explain what this command does here
		'''
		super().__init__()

		self.robot = robot

	# Overrides
	def getRequirements(self):
		return set( [self.robot.climber] )

	def initialize(self):
		self.robot.climber.stop()
		self.robot.climber.set_brake(True)
		# print("Stop.initialize: SET brake")

	def execute(self):
		pass

	def isFinished(self):
		return True

	def end(self, interrupted):
		self.robot.climber.stop()
		self.robot.climber.set_brake(True)
		# print("Stop.end      : SET brake")

	def interrupted(self):
		self.end()
