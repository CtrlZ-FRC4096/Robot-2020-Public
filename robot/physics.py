"""
Robot physics implementation for the pyfrc simulator

Taken from robotpy physics examples found here:
https://github.com/robotpy/examples/tree/master/physics/src
"""

### IMPORTS ###

import math

import pyfrc.physics.drivetrains
import pyfrc.physics.motor_cfgs
import pyfrc.physics.tankmodel

from pyfrc.physics.units import units

import hal.simulation

import const

### CLASSES ###

class Field:
	# TODO: this will be in a future pyfrc release

	def __init__(self):
		self.device = hal.SimDevice("Field2D")
		self.fx = self.device.createDouble("x", False, 0.0)
		self.fy = self.device.createDouble("y", False, 0.0)
		self.frot = self.device.createDouble("rot", False, 0.0)

		self.x = 0
		self.y = 0
		self.angle = 0

	def distance_drive(self, x, y, angle):
		"""
		Call this from your :func:`PhysicsEngine.update_sim` function.
		Will update the robot's position on the simulation field.

		This moves the robot some relative distance and angle from
		its current position.

		:param x:     Feet to move the robot in the x direction
		:param y:     Feet to move the robot in the y direction
		:param angle: Radians to turn the robot
		"""
		# TODO: use wpilib kinematics?

		self.angle += angle

		c = math.cos(self.angle)
		s = math.sin(self.angle)

		self.x += x * c - y * s
		self.y += x * s + y * c

		self.fx.set(self.x)
		self.fy.set(self.y)
		self.frot.set(math.degrees(self.angle))


class PhysicsEngine( ):
	"""
	Physics model for a simple two-motor WCD-style robot
	"""
	def __init__(self, physics_controller):
		pass
		# self.field = Field()

		# self.position = 0

		# # Motors
		# self.l_motor = hal.simulation.PWMSim(const.CAN_DRIVE_MOTOR_LEFT_1)
		# self.r_motor = hal.simulation.PWMSim(const.CAN_DRIVE_MOTOR_RIGHT_1)

		# self.motor = hal.simulation.PWMSim(4)

		# # Change these parameters to fit your robot!
		# bumper_width = 3.25 * units.inch

		# self.drivetrain = pyfrc.physics.tankmodel.TankModel.theory(
		# 	pyfrc.physics.motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
		# 	100 * units.lbs,                    # robot mass
		# 	10.71,                              # drivetrain gear ratio
		# 	2,                                  # motors per side
		# 	22 * units.inch,                    # robot wheelbase
		# 	23 * units.inch + bumper_width * 2, # robot width
		# 	32 * units.inch + bumper_width * 2, # robot length
		# 	6 * units.inch,                     # wheel diameter
		# )


	def update_sim(self, now, tm_diff):
		pass
		# l_motor_val = self.l_motor.getSpeed()
		# r_motor_val = self.r_motor.getSpeed()

		# x, y, rotation = self.drivetrain.get_distance(l_motor_val, r_motor_val, tm_diff)
		# self.field.distance_drive(x, y, rotation)

		# # update position (use tm_diff so the rate is constant)
		# self.position += self.motor.getSpeed() * tm_diff * 3
