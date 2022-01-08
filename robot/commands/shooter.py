"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020-2021
Code for robot "Krono-Z"
contact@team4096.org
"""


#long dist
# ance accurate shots
#table = [(75,8,27.15),(75,10,22.19),(75,12,18.59),(75,14,17.75), (75,16,17.31), (70,8,27.6), (70,10,23.1), (70,12,21.19), (70,14,20.56), (70,16.18.51), (65,12.23.46), (65,10,24.31), (65,8,28.6), (60,8,28.31), (55,6,33.45)]

#short range outer port shots
#table = [(60,3,50.69), (65,3,50.69), (70,3,50.69), (75,3,50.69), (60,4,49.41), (65,4,49.41), (70,4,49.41), (60,5,42.29), (65,5,42.29), (70,5,42.29), (75,5,42.29), (60,6,37.6), (65,6,37.6), (70,6,37.6), (75,6,37.6), (60,7,37.9), (65,7,37.9), (70,7,34.31), (75,7,34.31), (60,8,28.47)]

# target_shooter_speed = numpy.interp(current_distance, data_distances, data_shooter_speeds)
# target_hood_angle = numpy.interp(current_distance, data_distances, data_hood_angles)

# This is to help vscodem
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

# from wpilib.command import Command, CommandGroup, WaitCommand
from commands2 import Command

import const
import subsystems.shooter
import networktables

class Run_Percent(Command):
	def __init__(self, robot: "Robot", speed):
		super().__init__()
		self.robot = robot

		self.speed = speed

		self.hasRequirement(self.robot.shooter)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.shooter] )

	# Methods
	def execute(self):
		if callable(self.speed):
			self.robot.shooter.run_percent(self.speed())
		else:
			self.robot.shooter.run_percent(self.speed)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.shooter.stop()

	def interrupted(self):
		self.end()

class Run_Velocity(Command):
	def __init__(self, robot: "Robot", velocity):
		super().__init__()
		self.robot = robot

		self.velocity = velocity

		self.is_callable = callable(velocity)

		self.hasRequirement(self.robot.shooter)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.shooter] )

	# Methods
	def initialize(self):
		if not self.is_callable:
			self.robot.shooter.set_velocity(self.velocity)

	def execute(self):
		if self.is_callable:
			self.robot.shooter.set_velocity(self.velocity)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.shooter.stop()

	def interrupted(self):
		self.end()

class Stop(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()
		self.robot = robot
		self.hasRequirement(self.robot.shooter)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.shooter] )

	# Methods
	def execute(self):
		self.robot.shooter.stop()

	def isFinished(self):
		return True

	def end(self, interrupted):
		self.robot.shooter.stop()

	def interrupted(self):
		self.end()


class Set_Hood_Angle(Command):
	def __init__(self, robot: "Robot", angle):
		'''
		initializes tank drive movement.
		:param robot: the robot object
		:param angle: angle to set servo to
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.shooter)
		# self.setInterruptible(True)

		self.angle = angle

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.shooter] )

	# Methods
	def execute(self):
		"""
		Get r and y values from our joystick axes and send them to subsystem
		"""
		servo_value = self.angle / 360
		self.robot.hood.servo1.set(servo_value)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.hood.servo1.set(0)

	def interrupted(self):
		self.end()


class Shoot_N_Balls(Command):
	def __init__(self, robot: "Robot", velocity=0, nBalls=0):
		super().__init__()

		self.robot = robot
		self.velocity = velocity
		self.nBalls = nBalls

		self.hasRequirement(self.robot.shooter)
		# self.setInterruptible(True)
		# self.setTimeout(10)	# For now... probably remove later

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.shooter] )

	# Methods
	def execute(self):
		self.robot.shooter.set_velocity(self.velocity)

	def isFinished(self):
		if self.isTimedOut():
			return True
		return False

	def end(self, interrupted):
		self.robot.shooter.stop()

	def interrupted(self):
		self.end()