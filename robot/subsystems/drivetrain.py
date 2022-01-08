"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020-2021
Code for robot "Krono-Z"
contact@team4096.org
"""

# This is to help vscode
from typing import TYPE_CHECKING
from wpilib._wpilib import PWMSpeedController

from wpilib.interfaces._interfaces import SpeedController
if TYPE_CHECKING:
	from robot import Robot

import math
import sys

import commands2
from commands2 import Subsystem

import ctre
import wpilib
import wpilib.controller
import wpilib.drive
import wpilib.kinematics
import wpilib.geometry
import wpilib.trajectory
import wpimath

from wpimath.kinematics import DifferentialDriveWheelSpeeds

import const

### CLASSES ###

class Drivetrain(Subsystem):

	def __init__(self, robot: "Robot"):

		super().__init__()

		self.robot = robot

		class Gyro(wpilib.ADXRS450_Gyro): # nonsense needed because the gyro is mounted upside down
			def get(self):
				return -super().get()
			def getAngle(self) -> float:
				return -super().getAngle()
			def getRate(self) -> float:
				return -super().getRate()

		# Gyro
		self.gyro = Gyro()


		# DT encoders
		self.encoder_left = wpilib.Encoder(const.DIO_DRIVE_ENC_LEFT_1, const.DIO_DRIVE_ENC_LEFT_2, reverseDirection = const.DRIVE_ENCODER_LEFT_REVERSED)
		self.encoder_right = wpilib.Encoder(const.DIO_DRIVE_ENC_RIGHT_1, const.DIO_DRIVE_ENC_RIGHT_2, reverseDirection = const.DRIVE_ENCODER_RIGHT_REVERSED)

		self.encoder_left.setDistancePerPulse(1 / const.DRIVE_TICKS_PER_METER_LEFT)
		self.encoder_right.setDistancePerPulse(1 / const.DRIVE_TICKS_PER_METER_RIGHT)

		# Rotate PID
		# These values were from Midwest
		# self.rotate_kP = 0.009
		# self.rotate_kI = 0.0001
		# self.rotate_kD = 0.0006

		# These values are post-Midwest in-shop
		self.rotate_kP = 0.01
		self.rotate_kI = 0.01
		self.rotate_kD = 0.0008

		self.rotate_pid = wpilib.controller.PIDController(
			self.rotate_kP,
			self.rotate_kI,
			self.rotate_kD,
		)

		self.rotate_pid.setTolerance(1.0)
		self.rotate_pid.enableContinuousInput(-360, 360)

		# Drive distance PID - values are from 2018 (will need tweaking)
		self.distance_kP = 0.35
		self.distance_kI = 0.0
		self.distance_kD = 0.0

		self.drive_distance_pid = wpilib.controller.PIDController(
			self.distance_kP,
			self.distance_kI,
			self.distance_kD,
		)

		self.drive_distance_pid.setTolerance(0.05) # in feet
		# self.drive_distance_pid.setContinuous(False)

		# Differential Drive object - 3 MiniCIM's with Victor SPs per side
		self.motor_left1 = ctre.WPI_VictorSPX(const.CAN_DRIVE_MOTOR_LEFT_1)
		self.motor_left2 = ctre.WPI_VictorSPX(const.CAN_DRIVE_MOTOR_LEFT_2)
		self.motor_left3 = ctre.WPI_VictorSPX(const.CAN_DRIVE_MOTOR_LEFT_3)

		self.motor_left1.setInverted(False)
		self.motor_left2.setInverted(False)
		self.motor_left3.setInverted(False)

		self.motor_group_left = wpilib.SpeedControllerGroup(self.motor_left1, self.motor_left2, self.motor_left3)

		self.motor_right1 = ctre.WPI_VictorSPX(const.CAN_DRIVE_MOTOR_RIGHT_1)
		self.motor_right2 = ctre.WPI_VictorSPX(const.CAN_DRIVE_MOTOR_RIGHT_2)
		self.motor_right3 = ctre.WPI_VictorSPX(const.CAN_DRIVE_MOTOR_RIGHT_3)

		self.motor_right1.setInverted(False)
		self.motor_right2.setInverted(False)
		self.motor_right3.setInverted(False)

		self.motor_group_right = wpilib.SpeedControllerGroup(self.motor_right1, self.motor_right2, self.motor_right3)

		self.drive = wpilib.drive.DifferentialDrive(self.motor_group_left, self.motor_group_right)


		self.motor_left1.setNeutralMode(ctre.NeutralMode.Brake)
		self.motor_left2.setNeutralMode(ctre.NeutralMode.Brake)
		self.motor_left3.setNeutralMode(ctre.NeutralMode.Brake)

		self.motor_right1.setNeutralMode(ctre.NeutralMode.Brake)
		self.motor_right2.setNeutralMode(ctre.NeutralMode.Brake)
		self.motor_right3.setNeutralMode(ctre.NeutralMode.Brake)

		# #Drive correction
		# self.r = 0.0
		# self.corrected_rotation = 0.0
		# self.y = 0.0
		# self.x = 0.0

		self.sensitivity = 1.0

		self._was_correcting = False
		self._correction_start_angle = None

		self.last_right_enc = 0
		self.last_left_enc = 0

		self.kinematics = wpilib.kinematics.DifferentialDriveKinematics(const.TRACK_WIDTH)
		self.odometry = wpilib.kinematics.DifferentialDriveOdometry( wpilib.geometry.Rotation2d.fromDegrees( self.get_heading() ) )

		# self.old_encoder_speed = 0
		# self.encoder_speeds = [0] * 50
	# FUNCTION OVERRIDES

	def periodic(self):
		self.odometry.update( wpilib.geometry.Rotation2d.fromDegrees( -self.gyro.getAngle() ), self.encoder_left.getDistance(), self.encoder_right.getDistance() )

		# self.encoder_speed = (self.encoder_left.getRate() + self.encoder_right.getRate()) / 2
		# self.encoder_speeds.append(self.encoder_speed)
		# self.encoder_speeds.pop(0)
		#print(self.encoder_speed, (self.encoder_speed - self.old_encoder_speed) * 50)

		# print(f"{self.encoder_speed: .3f} {(self.encoder_speed - self.encoder_speeds[0]) : .3f}")

		# self.old_encoder_speed = self.encoder_speed


	def getPose( self ):
		return self.odometry.getPose()


	def tankDriveVolts( self, left_volts, right_volts ):
		self.motor_group_left.setVoltage( left_volts )
		self.motor_group_right.setVoltage( -right_volts )
		self.drive.feed()


	# FUNCTIONS

	def get_heading(self):
		return math.remainder(self.gyro.getAngle(), 360)

	def get_drive_encoders_distance(self):
		distance_right = self.encoder_right.getDistance()
		distance_left = self.encoder_left.getDistance()
		avg_distance = (distance_right + distance_left) / 2.0

		return avg_distance

	# def get_turn_rate(self):
		# return m_gyro.getRate() * -1.0
	#	return self.gyro.getRate() * (Drive)

	def use_drive_distance_pid_output(self, output):
		min_output = const.DRIVE_MIN_OUTPUT_TANK

		if output < 0:
			output = min(output, -min_output)
		elif output > 0:
			output = max(output, min_output)

		self.drive_with_tank_values(0, output, 0)


	def update_drive_distance_pid(self, p = None, i = None, d = None):
		'''
		Updates the PID coefficients
		'''
		if p:
			self.distance_kP = p
		if i:
			self.distance_kI = i
		if d:
			self.distance_kD = d

		print('Drive Distance PID updated: {0:.2f}, {1:.2f}, {2:.2f}'.format(self.distance_kP, self.distance_kI, self.distance_kD))
		self.drive_distance_pid.setPID(self.distance_kP, self.distance_kI, self.distance_kD)


	def use_rotate_pid_output( self, output ):
		# Make sure we're outputing at least enough to move the robot at all
		min_val = 0.4

		output1 = output

		if output > 0:
			output += min_val
		elif output < 0:
			output -= min_val

		output2 = output

		output = max(-1, min(1, output))

		print(output1, output2, output)
		# min_output = const.DRIVE_MIN_ROTATION_OUTPUT_TANK

		# if output < 0:
		# 	output = min( output, -min_output )
		# 	output = max( output, -.9 )
		# elif output > 0:
		# 	output = max( output, min_output )
		# 	output = min( output, .9 )

		#wpilib.SmartDashboard.putString( 'Drive Rotate PID Output: ', '{0:.3f}'.format( output ) )
		self.drive.tankDrive(output, -output, False)
		# self.drive_with_tank_values( output, 0, 0 )


	def run(self, rotation):
		self.drive_with_tank_values(rotation, 0, 0)

	def drive_with_tank_joysticks(self, left_axis, right_axis):
		"""
		needs work, and support for correction.
		"""
		self.drive.arcadeDrive(left_axis(), right_axis())

	def set_sensitivity (self, sensitivity):
		self.sensitivity = sensitivity

	def _get_corrected_rotation_gyro(self, rotation_value, y_value, strafe_value):
		"""
		Does drive correction using drivetrain gyro (not the encoders)
		"""
		if not const.DRIVE_CORRECTION_ENABLED:
			self.robot.nt_robot.putString('Drive Correction: ', 'DISABLED')
			self._correction_start_angle = None
			return rotation_value

		if abs(y_value) > 0.025 and abs(rotation_value) < const.DRIVE_CORRECTION_ROTATION_THRESHOLD:
			# This means driver is moving forward/backward or strafing, but NOT rotating
			if self._correction_start_angle is None:
				# We were not correcting last time, so save gyro angle and start correcting to that
				print('Resetting gyro!')
				self.gyro.reset()
				self._correction_start_angle = get_heading()
				return rotation_value

			else:
				# We're correcting so adjust rotation
				if y_value < 0:
					tmp_correction_proportion = const.DRIVE_CORRECTION_PROPORTION_FORWARD
				else:
					tmp_correction_proportion = const.DRIVE_CORRECTION_PROPORTION_REVERSE

				correction_amt = (self._correction_start_angle - (get_heading())) * -tmp_correction_proportion
				rotation_value += correction_amt
				self.robot.nt_robot.putString('Drive Correction: ', 'ACTIVE - {0:.4f}'.format(correction_amt))
				self.robot.nt_robot.putNumber('Drive Correction Int: ', 1.0)
		else:
			self.robot.nt_robot.putString('Drive Correction: ', 'INACTIVE')
			self.robot.nt_robot.putNumber('Drive Correction Int: ', 0.0)
			self._correction_start_angle = None

		return rotation_value


	def drive_with_tank_values(self, rotation_value, y_value, strafe_value = 0):
		# print('r1 = {0:.2f}, y1 = {1:.2f}'.format(rotation_value, y_value))
		self.r = rotation_value
		self.y = y_value # * self.sensitivity

		if const.DRIVE_CORRECTION_ENABLED:
			corrected_rotation_value = self._get_corrected_rotation_gyro(rotation_value, y_value, strafe_value)

			self.corrected_rotation = corrected_rotation_value
			# print('r2 = {0:.2f}, y2 = {1:.2f}'.format(corrected_rotation_value, y_value))
			self.drive.arcadeDrive(y_value, corrected_rotation_value)
		else:
			# self.drive.arcadeDrive(y_value, rotation_value)
			# max_volts = 11 / 12
			# y_value = min()
			s = 1
			# self.tankDriveVolts(y_value +  s * rotation_value, y_value - s * rotation_value)
			self.drive.tankDrive(y_value +  s * rotation_value, y_value - s * rotation_value)


	def get_wheel_speeds(self):
		return DifferentialDriveWheelSpeeds(self.encoder_left.getRate(),self.encoder_right.getRate())

	def reset_encoders(self):

		self.encoder_right.reset()
		self.encoder_left.reset()

	def reset_odometry(self, pose):
		self.reset_encoders()
		self.odometry.resetPosition(pose, wpimath.geometry.Rotation2d.fromDegrees(self.get_heading()))

	def stop(self):
		self.drive_with_tank_values(0, 0, 0)
		self.drive.stopMotor()

	def log(self):
		'''
		logs various things about the drivetrain
		'''
		# print( 'Drive values: Y = {0:.2f}, R = {1:.2f}'.format( self.y, self.r ) )

		self.robot.nt_robot.putNumber( 'Drive Y', self.y )
		self.robot.nt_robot.putNumber( 'Drive R', self.r )

		self.robot.nt_robot.putString( 'Gyro', '{0:.2f}'.format( self.get_heading() ) )

		self.robot.nt_robot.putString( 'DT Encoder Left', '{0:.2f}'.format( self.encoder_left.getDistance() ) )
		self.robot.nt_robot.putString( 'DT Encoder Right', '{0:.2f}'.format( self.encoder_right.getDistance() ) )
		# print(self.encoder_left.get(), self.encoder_right.get(), self.gyro.getAngle())
		self.robot.nt_robot.putString( 'DT Encoder Distance', '{0:.2f}'.format( self.get_drive_encoders_distance() ) )
		self.robot.nt_robot.putValue("DT Pose", str(self.getPose()))
		self.robot.nt_robot.putValue("DT Wheel Speeds", str(self.get_wheel_speeds()))
		# print(self.encoder_left.getRaw())
		# print(self.encoder_right.getRaw())
		# print(self.getPose())


