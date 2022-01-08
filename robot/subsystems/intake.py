"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020-2021
Code for robot "------"
contact@team4096.org
"""

# This is to help vscode
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

import math
import time
import sys

import commands2
from commands2 import Subsystem
import wpilib
import networktables

import ctre

import const

class Intake(Subsystem):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.motor = ctre.WPI_TalonSRX(const.CAN_INTAKE)

		self.motor.setNeutralMode(ctre.NeutralMode.Brake)

		self.piston = wpilib.DoubleSolenoid(
			const.PCM_SOLENOID_INTAKE_DEPLOY_1,
			const.PCM_SOLENOID_INTAKE_DEPLOY_2,
		)

	def run(self, speed):
		self.motor.set(speed)

	def intake(self):
		self.run(1)

	def outtake(self):
		self.run(-.5)

	def stop(self):
		self.run(0)

	def up(self):
		self.piston.set(self.piston.Value.kForward)

	def down(self):
		self.piston.set(self.piston.Value.kReverse)

	def log(self):
		pass
		# self.robot.nt_robot.putBoolean('Blah', True)

