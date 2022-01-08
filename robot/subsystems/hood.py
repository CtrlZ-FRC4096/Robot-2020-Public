"""
Ctrl-Z FRC Team 4096
FIRST Robotic Competition 2020
Code for robot "Krono-Z"
contact@team4096.org
"""

# This is to help vscode
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

import math
import sys

import wpilib
import wpilib.controller

import commands2
from commands2 import Subsystem

import const

### CLASSES ###

class Hood(Subsystem):

	def __init__(self, robot: "Robot"):

		super().__init__()

		self.robot = robot

		# self.encoder_left = wpilib.Encoder(const.DIO_DRIVE_ENC_LEFT_1, const.DIO_DRIVE_ENC_LEFT_2, reverseDirection = const.DRIVE_ENCODER_LEFT_REVERSED)
		# self.encoder_left.setDistancePerPulse(1 / const.DRIVE_TICKS_PER_FOOT)

		# Rotate PID
		self.rotate_kP = -0.1
		self.rotate_kI = 0.005
		self.rotate_kD = 0.0

		self.rotate_pid = wpilib.controller.PIDController(
			self.rotate_kP,
			self.rotate_kI,
			self.rotate_kD,
		)

		self.rotate_pid.setTolerance(.3)
		#change ^^^ to higher tolerance for PID
		self.rotate_pid.disableContinuousInput()

		# Motors
		self.servo1 = wpilib.PWMSpeedController(0)

		# set up potentiometer
		self.pot = wpilib.AnalogPotentiometer(
			1,
			-157.805,
			67.6339
		)

	def stop(self):
		pass


	def log(self):
		'''
		logs various things about this subsystem
		'''
		pass
		self.robot.nt_robot.putString( 'Hood POS', '{0:.2f}'.format( self.pot.get() ) )
		# print('Hood Pos: {0:.4f}'.format( self.pot.get()  ) )
