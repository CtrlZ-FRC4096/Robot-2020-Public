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

import wpilib
import commands2
from commands2 import Subsystem
import networktables

import const

import ctre

class Climber(Subsystem):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.motor0 = ctre.WPI_VictorSPX(const.CAN_CLIMBER_0)
		self.motor1 = ctre.WPI_VictorSPX(const.CAN_CLIMBER_1)

		self.motor1.follow(self.motor0)

		self.motor0.setNeutralMode(ctre.NeutralMode.Brake)
		self.motor1.setNeutralMode(ctre.NeutralMode.Brake)

		# Solenoids
		self.brake_solenoid = wpilib.DoubleSolenoid(
			const.PCM_SOLENOID_CLIMBER_BRAKE_1,
			const.PCM_SOLENOID_CLIMBER_BRAKE_2,
		)

		self.deploy_solenoid = wpilib.DoubleSolenoid(
			const.PCM_SOLENOID_CLIMBER_DEPLOY_1,
			const.PCM_SOLENOID_CLIMBER_DEPLOY_2,
		)

		self.is_deployed = False

		# class Dummy_Limit_Switch:
		# 	def get(self):
		# 		return True
		# self.top_limit_switch = Dummy_Limit_Switch()
		# self.bottom_limit_switch = Dummy_Limit_Switch()

		# When fully extended
		self.top_limit_switch = wpilib.DigitalInput(const.DIO_CLIMBER_TOP_LIMIT)
		# When fully retracted
		self.bottom_limit_switch = wpilib.DigitalInput(const.DIO_CLIMBER_BOTTOM_LIMIT)


	def wind(self, speed=.7):
		speed = -abs(speed)
		self.motor0.set(speed)

	def unwind(self, speed=0.7):
		speed = abs(speed)
		self.motor0.set(speed)

	def stop(self):
		self.motor0.set(0)

	def deploy(self):
		self.deploy_solenoid.set( wpilib.DoubleSolenoid.Value.kReverse )
		self.is_deployed = True

	def undeploy(self):
		self.deploy_solenoid.set( wpilib.DoubleSolenoid.Value.kForward )
		self.is_deployed = False

	def set_brake_on(self):
		self.brake_solenoid.set( wpilib.DoubleSolenoid.Value.kForward )

	def set_brake_off(self):
		self.brake_solenoid.set( wpilib.DoubleSolenoid.Value.kReverse )

	def set_brake(self, state: bool):
		if state:
			self.set_brake_on()
		else:
			self.set_brake_off()

	def log(self):
		pass
		self.robot.nt_robot.putBoolean('Climber Top Switch', self.top_limit_switch.get())
		self.robot.nt_robot.putBoolean('Climber Bottom Switch', self.bottom_limit_switch.get())
		# print( 'ty = {0:.2f}'.format( self.nt_table.getNumber('ty', 0)) )
