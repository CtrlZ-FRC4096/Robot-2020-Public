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

import math
import sys

from commands2 import Subsystem

import wpilib
import wpilib.controller
import wpilib.drive

import const

import ctre

### CLASSES ###

class Turret(Subsystem):

	def __init__(self, robot: "Robot"):

		super().__init__()

		self.robot = robot

		# self.encoder = wpilib.Encoder(const.DIO_TURRET_ENCODER_1, const.DIO_TURRET_ENCODER_2, reverseDirection = const.TURRET_ENCODER_REVERSED)
		# self.encoder.setDistancePerPulse(1 / const.DRIVE_TICKS_PER_FOOT)

		# # Rotate PID
		# self.rotate_kP = 0.02
		# self.rotate_kI = 0.0
		# self.rotate_kD = 0.0

		# self.rotate_pid = wpilib.controller.PIDController(
		# 	self.rotate_kP,
		# 	self.rotate_kI,
		# 	self.rotate_kD,
		# )

		# self.rotate_pid.setTolerance(2.0)
		# self.rotate_pid.enableContinuousInput(-360, 360)

		self.pot = wpilib.AnalogPotentiometer(
			2,
			395,
			-305
		)

		# Motors
		self.motor = ctre.WPI_TalonSRX(const.CAN_TURRET)

		self.motor.setNeutralMode(ctre.NeutralMode.Brake)

		# self.motor.configSelectedFeedbackSensor(ctre.FeedbackDevice.Analog)


	def run(self, value):
		self.motor.set(value)
		# print(self.motor.getSelectedSensorPosition())

	def rotate_left(self, speed):
		self.run(-speed)

	def rotate_right(self, speed):
		self.run(speed)

	def stop(self):
		self.motor.set(0)

	# def use_rotate_pid_output( self, output ):
	# 	# Make sure we're outputing at least enough to move the robot at all
	# 	min_output = const.TURRET_MIN_ROTATION_OUTPUT

	# 	if output < 0:
	# 		output = min( output, -min_output )
	# 		output = max( output, -1.0 )
	# 	elif output > 0:
	# 		output = max( output, min_output )
	# 		output = min( output, 1.0 )

	# 	#wpilib.SmartDashboard.putString( 'Turret Rotate PID Output: ', '{0:.3f}'.format( output ) )
	# 	# print('Turret Rotate PID output = {0:.3f}'.format( output ) )

	# 	self.rotate( output )





	def log(self):
		'''
		logs various things about this subsystem
		'''
		#print(f"Turret sensor position {self.pot.get()}")
		# self.robot.nt_robot.putString( 'Hood Servo', '{0:.2f}'.format( self.hood_servo.getAngle() ) )

