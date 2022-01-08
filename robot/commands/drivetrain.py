"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020-2021
Code for robot "Krono-Z"
contact@team4096.org
"""

# This is to help vscode
import math
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from robot import Robot

# from wpilib.command import Command, CommandGroup, WaitCommand
import commands2
from commands2 import Command, CommandBase

import const
import subsystems.drivetrain
import networktables

class Drive_With_Tank_Values(Command):
	def __init__(self, robot: "Robot", get_r, get_y):
		'''
		initializes tank drive movement.
		:param robot: the robot object
		:param get_r: Used to get the x angle, function that determines y direction
		:param get_y: Used to get the y angle, function that determines the y value and direction
		rotation and direction of rotation. Z value must be given if it separate from the joystick.
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drive)
		# self.setInterruptible(True)

		self.get_r = get_r
		self.get_y = get_y

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drive] )

	# Methods
	def execute(self):
		"""
		Get r and y values from our joystick axes and send them to subsystem
		"""
		# print(self.get_r(), self.get_y())
		self.robot.drive.drive_with_tank_values(self.get_r() * const.DRIVE_ROTATION_MULTIPLIER, self.get_y())
		# self.robot.drive.drive_with_tank_joysticks(self.get_y, self.get_r)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.drive.stop()

	def interrupted(self):
		self.end()

class Drive_Distance(Command):
	"""
	Drive specific distance. "distance" is in FEET
	"""
	def __init__ (self, robot: "Robot", distance, max_speed = None):
		super().__init__()

		self.robot = robot
		self.distance = distance		# in FEET - ie 5.5 = 5 feet 6 inches
		self.distance_traveled = 0.0
		# self.setInterruptible(False)

		self.hasRequirement(self.robot.drive)
		# self.setTimeout(6)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drive] )

	# Methods
	def initialize(self):
		self.robot.drive.encoder_left.reset()
		self.robot.drive.encoder_right.reset()

		print("Setpoint:", self.distance)
		self.robot.drive.drive_distance_pid.setSetpoint(self.distance)


	def execute(self):
		output = self.robot.drive.drive_distance_pid.calculate(self.robot.drive.get_drive_encoders_distance())
		self.robot.drive.use_drive_distance_pid_output(output)


	def isFinished(self):
		on_target = self.robot.drive.drive_distance_pid.getPositionError() < 0.1
		timed_out = self.isTimedOut()

		if timed_out:
			print('Drive Distance Timed out!')
			return True
		if on_target:
			self.distance_traveled = self.robot.drive.get_drive_encoders_distance()
			print('Drive Distance On Target! {0:.2f}'.format(self.distance_traveled))
			return True

		# keep going
		return False

	def end(self, interrupted):
		pass
		#print("Ending Drive Distance")

	def interrupted(self):
		self.end()


class Rotate_To_Angle(Command):
	def __init__(self, robot: "Robot", angle):
		super().__init__()

		self.robot = robot
		self.angle = angle

		self.hasRequirement(self.robot.drive)
		# self.setInterruptible(True)
		# self.setTimeout(5)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drive] )

	# Methods
	def initialize(self):
		self.robot.drive.gyro.reset()
		print('gyro start = {0:.2f}'.format(self.robot.drive.gyro.getAngle()))
		self.robot.drive.rotate_pid.setSetpoint(self.angle)

	def execute(self):
		output = self.robot.drive.rotate_pid.calculate(self.robot.drive.gyro.getAngle())
		self.robot.drive.use_rotate_pid_output(output)

	def isFinished(self):
		on_target = self.robot.drive.rotate_pid.getPositionError() < 1.0
		timed_out = self.isTimedOut()

		# print('gyro = {0:.2f}'.format(self.robot.drive.gyro.getAngle()))

		if timed_out:
			print('Rotate To Angle Timed out!')
			return True
		if on_target:
			print('Rotate On Target! {0:.2f}'.format(self.robot.drive.gyro.getAngle()))
			return True

		# keep going
		return False

	def end(self, interrupted):
		pass
		# self.robot.drive.rotate_pid.disable()

	def interrupted(self):
		self.end()


class Stop(Command):
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drive)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drive] )

	# Methods
	def execute(self):
		print( 'Stop' )
		self.robot.drive.drive_with_tank_values(0, 0, 0)

	def isFinished(self):
		return True


class Toggle_Correction(Command):
	# Switches between drive correction and no drive correction
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drive)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drive] )

	# Methods
	def execute(self):
		const.DRIVE_CORRECTION_ENABLED = not const.DRIVE_CORRECTION_ENABLED

	def isFinished(self):
		return True

#@commands2.Command.withTimeout(3)
#class Drive_Time(Command):
	def __init__ (self, robot: "Robot", time, speed):
		super().__init__()

		self.robot = robot
		self.speed = -speed

		self.hasRequirement(self.robot.drive)
		# self.setInterruptible(True)
		#self.withTimeout(time)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drive] )

	# Methods
	def execute(self):
		print("Driving")
		self.robot.drive.drive_with_tank_values(0, self.speed)

	def isFinished(self):
		timed_out = self.isTimedOut()

		return timed_out


class Rotate_To_Limelight_Target(Command):
	"""
	This uses the drivetrain's rotate PID to rotate the robot and align it to
	a vision target seen by the Limelight. If no target is visible it will
	do nothing.

	**Arguments:**

		:``robot``: `Robot` robot object

	**Keyword Arguments:**

		None
	"""

	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drive)
		# self.setInterruptible(True)
		# self.setTimeout(5)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drive] )

	# Methods
	def initialize(self):
		self.robot.drive.rotate_pid.setSetpoint(0)

	def execute(self):
		self.robot.limelight.turn_leds_on()

		target_angle = self.robot.limelight.get_angle_to_target()

		speed = math.copysign(3, target_angle)
		self.robot.drive.tankDriveVolts(speed, speed)

		# print( 'target_angle = {0:.2f}'.format( target_angle ))

		# output = self.robot.drive.rotate_pid.calculate(target_angle)
		# print( 'output = {0:.2f}'.format( output ))

		# self.robot.drive.use_rotate_pid_output(output)

	def isFinished(self):
		# print( 'error: {0:.2f}'.format( self.robot.drive.rotate_pid.getPositionError() ) )
		print( 'angle: {0:.2f}'.format( self.robot.limelight.get_angle_to_target() ) )

		# # on_target = abs(self.robot.drive.rotate_pid.getPositionError()) < 1.0
		# on_target = abs(self.robot.limelight.get_angle_to_target()) < 1.0

		# if on_target:
		# 	print('Rotate On Target! {0:.2f}'.format(self.robot.limelight.get_angle_to_target()))
		# 	return True

		# keep going
		return False

	def end(self, interrupted):
		self.robot.limelight.turn_leds_off()
		pass


class Find_And_Rotate_To_Limelight_Target(Command):
	"""
	This is the same as Rotate_To_Limelight_Target above, except that if it cannot see
	a target when it begins, it will rotate the robot in a circle until one is
	found. At that point it behaves just like the other command.

	**Arguments:**

		:``robot``: `Robot` robot object

	**Keyword Arguments:**

		None
	"""

	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drive)
		# self.setInterruptible(True)
		# self.setTimeout(5)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drive] )

	# Methods
	def initialize(self):
		self.robot.drive.gyro.reset()
		self.robot.drive.rotate_pid.setSetpoint(0)

	def execute(self):
		self.robot.limelight.turn_leds_on()

		if self.robot.limelight.is_target_visible():
			angle_to_target = self.robot.limelight.get_angle_to_target()
		else:
			# No target visible, so rotate to find it
			angle_to_target = 35

		print( 'angle_to_target = {0:.2f}'.format(angle_to_target))

		if abs( angle_to_target ) < 2.0:
			output = 0
		else:
			output = self.robot.drive.rotate_pid.calculate(angle_to_target)

		self.robot.drive.use_rotate_pid_output(output)

	def isFinished(self):
		on_target = abs(self.robot.drive.rotate_pid.getPositionError()) < 1.0
		on_target = False
		# on_target = abs(self.robot.limelight.get_angle_to_target()) < 0.5
		timed_out = self.isTimedOut()

		# print('gyro = {0:.2f}'.format(self.robot.drive.gyro.getAngle()))

		if timed_out:
			print('Rotate To Angle Limelight Timed out!')
			self.robot.limelight.turn_leds_off()
			return True
		if on_target:
			print('Rotate On Target! {0:.2f}'.format(self.robot.limelight.get_angle_to_target()))
			self.robot.limelight.turn_leds_off()
			return True

		# keep going
		return False

	def end(self, interrupted):
		# self.robot.limelight.set_led_mode(const.LIMELIGHT_LED_MODE_OFF)
		pass

	def interrupted(self):
		self.end()
