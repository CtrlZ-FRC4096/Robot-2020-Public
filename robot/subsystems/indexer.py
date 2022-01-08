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

import const
import ctre

class Indexer(Subsystem):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.motor_bottom = ctre.WPI_VictorSPX(const.CAN_INDEXER_BOTTOM)
		self.motor_left = ctre.WPI_VictorSPX(const.CAN_INDEXER_LEFT)
		self.motor_right = ctre.WPI_VictorSPX(const.CAN_INDEXER_RIGHT)

		self.motor_bottom.setNeutralMode(ctre.NeutralMode.Brake)
		self.motor_left.setNeutralMode(ctre.NeutralMode.Brake)
		self.motor_right.setNeutralMode(ctre.NeutralMode.Brake)

		self.motor_right.setInverted(True)
		self.motor_bottom.setInverted(True)

	def run_left(self, speed):
		self.motor_left.set(speed)

	def run_right(self, speed):
		self.motor_right.set(speed)

	def run_bottom(self, speed):
		self.motor_bottom.set(speed)

	def intake(self):
		# self.run_bottom(0.5)
		self.run_left(0.2)
		self.run_right(0.1)

	def feed(self):
		self.run_bottom(0.5)
		self.run_left(0.4)
		self.run_right(.25)

	def outtake(self):
		self.run_bottom(-0.15)
		self.run_left(-0.3)
		self.run_right(-0.3)

	def shuffle(self):
		self.run_left(0.25)
		self.run_right(-0.25)

	def stop(self):
		self.run_bottom(0)
		self.run_left(0)
		self.run_right(0)


	def log(self):
		pass
		# self.robot.nt_robot.putBoolean('Blah', True)

