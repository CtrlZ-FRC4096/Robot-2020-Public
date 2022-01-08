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

import commands2
from commands2 import Subsystem
import wpilib
import wpilib.controller
import networktables

import const

import ctre

### CLASSES ###

class Shooter(Subsystem):

	def __init__(self, robot: "Robot"):

		super().__init__()

		self.robot = robot

		# create motor controller objects
		self.motor0 = ctre.WPI_TalonSRX(const.CAN_SHOOTER_0)
		self.motor1 = ctre.WPI_TalonSRX(const.CAN_SHOOTER_1)

		# reset controllers to a known/factory state
		self.motor0.configFactoryDefault()
		self.motor1.configFactoryDefault()

		self.motor0.setNeutralMode(ctre.NeutralMode.Coast)
		self.motor1.setNeutralMode(ctre.NeutralMode.Coast)

		self.motor1.setInverted(True)

		self.motor1.follow(self.motor0) # make this aux for pid

		self.motor0.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder)
		self.motor0.setSensorPhase(True)

		self.motor0.configNominalOutputForward(0)
		self.motor0.configNominalOutputReverse(0)
		self.motor0.configPeakOutputForward(1)
		self.motor0.configPeakOutputReverse(0)


		self.motor0.config_kF(0, const.SHOOTER_KF)
		self.motor0.config_kP(0, const.SHOOTER_KP)
		self.motor0.config_kI(0, const.SHOOTER_KI)
		self.motor0.config_kD(0, const.SHOOTER_KD)
		self.motor0.config_IntegralZone(0, const.SHOOTER_IZ)


		# Current Limiting
		self.motor0.configPeakCurrentLimit(50, 0)
		self.motor0.configPeakCurrentDuration(800)
		self.motor0.configContinuousCurrentLimit(40)
		self.motor0.enableCurrentLimit(True)


		# TRY THIS - Voltage Compensation
		self.motor0.enableVoltageCompensation(True)
		self.motor0.configVoltageCompSaturation(12.0)

		# TRY THIS - smaller velocity measurement window
		self.motor0.configVelocityMeasurementPeriod( ctre.VelocityMeasPeriod.Period_2Ms )
		self.motor0.configVelocityMeasurementWindow(8)

		# # TRY THIS - bang bang
		# # The motor controller can only go at 100% or 0% power
		# self.motor0.configNominalOutputForward(0)
		# self.motor0.configNominalOutputReverse(0)
		# self.motor0.configPeakOutputForward(1)
		# self.motor0.configPeakOutputReverse(0)
		# self.motor0.config_kF(0, 0)
		# self.motor0.config_kP(0, 1000) # <- a big number
		# self.motor0.config_kI(0, 0)
		# self.motor0.config_kD(0, 0)




		# nt stuff
		self.pid_mode = False
		self.velocity = False

		self.is_at_speed = False

	def set_percent(self, speed):
		# self.motor0.set(ctre.ControlMode.)
		self.motor0.set(ctre.ControlMode.PercentOutput, speed)
		self.pid_mode = False
		self.velocity = 0
		# print(self.motor0.getSelectedSensorPosition(), self.motor0.getSelectedSensorVelocity())


	def stop(self):
		self.set_percent(0)

	def inc_velocity(self, inc):
		self.set_velocity(self.velocity + inc)

	def dec_velocity(self, dec):
		self.set_velocity(self.velocity - dec)

	def set_velocity(self, velocity):

		if velocity > const.SHOOTER_MAX_VELOCITY:
			print('The desired velocity is greater than the max velocity of the shooter.')

		self.motor0.set(ctre.ControlMode.Velocity, velocity)
		self.motor1.follow(self.motor0)

		self.pid_mode = True
		self.velocity = velocity
		self.atSpeed = False

	def periodic(self):

		self.is_at_speed = (
			self.pid_mode
			and self.motor0.getClosedLoopError() / const.SHOOTER_MAX_VELOCITY < 0.05
			)

		# # TRY THIS - Derivative Zone (Like Integral Zone)
		# if self.is_at_speed:
		# 	self.motor0.config_kD(40)
		# else:
		# 	self.motor0.config_kD(const.SHOOTER_KD)

	def log(self):
		'''
		logs various things about this subsystem
		'''
		self.robot.nt_robot.putBoolean("Using Shooter PID", self.pid_mode)
		self.robot.nt_robot.putNumber("Shooter Velocity", self.motor0.getSelectedSensorVelocity())
		self.robot.nt_robot.putNumber("Shooter Velocity Setpoint", self.velocity)
		self.robot.nt_robot.putBoolean("Shooter At Speed?", self.is_at_speed)
		# print("Shooter at speed" + str(self.motor0.getSelectedSensorVelocity()))
