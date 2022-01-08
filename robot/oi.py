"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020-2021
Code for robot "Krono-Z"
contact@team4096.org
"""

# This is to help vscode
from typing import TYPE_CHECKING

from wpilib._wpilib import SmartDashboard
if TYPE_CHECKING:
	from robot import Robot

import wpilib

###  IMPORTS ###

# Subsystems

# Commands
import commands.drivetrain
import commands.turret
import commands.hood
import commands.indexer
import commands.intake
import commands.shooter
import commands.climber
import commands.autonomous
# Controls
from customcontroller import XboxCommandController
from commands2.button import Button
from commands2 import ParallelCommandGroup

import const

## CONSTANTS ##

class OI:
	"""
	Operator Input - This class ties together controls and commands.
	"""

	# def why(self):
	# 	pass

	def __init__(self, robot: "Robot"):
		self.robot = robot

		# Controllers
		self.driver1 = XboxCommandController(0)
		self.driver2 = XboxCommandController(1)

		self.ddr = wpilib.Joystick(2)

		### Driving ###
		self.driver1.LEFT_JOY_Y.setInverted(True)
		self.driver1.LEFT_JOY_Y.setDeadzone(.1)

		self.driver1.RIGHT_JOY_X.setDeadzone(.1)


		def ddrxv():
			b1 = self.ddr.getRawButton(1)
			b2 = self.ddr.getRawButton(2)
			b3 = self.ddr.getRawButton(3)
			b4 = self.ddr.getRawButton(4)


			if b3 and not b4:
				if b1 or b2: return -.7
				return -1
			elif b4 and not b3:
				if b1 or b2: return .7
				return 1
			elif b1 or b2: return -0.01
			else: return 0



		def ddryv():
			b1 = self.ddr.getRawButton(1)
			b2 = self.ddr.getRawButton(2)
			b3 = self.ddr.getRawButton(3)
			b4 = self.ddr.getRawButton(4)


			if b1 and not b2:
				return .5
			elif b2 and not b1:
				return -.5
			else: return 0

		self.drive_command = commands.drivetrain.Drive_With_Tank_Values(
			self.robot,
			lambda: ddrxv() + self.driver1.RIGHT_JOY_X(),
			lambda: ddryv() + self.driver1.LEFT_JOY_Y(),
		)


		self.robot.drive.setDefaultCommand(self.drive_command)

		self.driver1.X.whenPressed(commands.drivetrain.Rotate_To_Limelight_Target(self.robot))
		self.driver1.X.whenReleased(self.drive_command)
		#

		#Auto

		#self.auto_command = commands.autonomous.Run_Pathweaver_Path(self.robot, 'Ball_Auto1.wpilib.json')
		#self.driver1.A.whenHeld(self.auto_command)

		### End Game ###

		# self.end_game_vibes = Button(lambda: 30 < self.robot.match_time < 32)

		# self.end_game_vibes.whenPressed(lambda: self.driver1.setRumble(1))
		# self.end_game_vibes.whenReleased(lambda: self.driver1.setRumble(0))


		### intaking ###

		self.driver2.LEFT_JOY_DOWN.whenPressed(commands.intake.Down(self.robot))
		self.driver2.LEFT_JOY_DOWN.whenReleased(commands.intake.Up(self.robot))

		self.driver2.A.whenHeld(commands.intake.In(self.robot))
		self.driver2.A.whenHeld(commands.indexer.In(self.robot))

		self.driver2.B.whenHeld(commands.intake.Out(self.robot))

		self.driver2.Y.whenHeld(commands.indexer.Out(self.robot))

		### shooting ###

		self.driver2.RIGHT_BUMPER.whenPressed(commands.shooter.Stop(self.robot))

		self.driver2.LEFT_BUMPER.whenPressed(
		 	ParallelCommandGroup(
		 		commands.hood.Set_Hood_Position(self.robot, 21.3),
		 		commands.shooter.Run_Velocity(self.robot, 78_000)
		 	)
		)

		self.driver2.LEFT_TRIGGER_AS_BUTTON.whenPressed(
			ParallelCommandGroup(
		 		commands.hood.Set_Hood_Position(self.robot, 25),
				commands.shooter.Run_Velocity(self.robot, 70_000)
			)
		)

		self.driver2.POV.UP.whenHeld(commands.hood.Set_Hood_Speed(self.robot, 0.8))
		self.driver2.POV.DOWN.whenHeld(commands.hood.Set_Hood_Speed(self.robot, -0.8))

		#self.driver2.POV.RIGHT.whenPressed(lambda: self.robot.shooter.inc_velocity(5_000))
		#self.driver2.POV.LEFT.whenPressed(lambda: self.robot.shooter.dec_velocity(5_000))
		self.driver2.POV.LEFT.whenHeld(commands.hood.Set_Hood_Position(self.robot, commands.hood.Find_Hood_Angle(robot)))

		# shoot
		self.driver2.RIGHT_TRIGGER_AS_BUTTON.whenHeld(commands.indexer.Feed(self.robot))

		### climbing ###

		self.driver1.POV.UP.whenPressed(commands.climber.Safe_Deploy(self.robot))
		self.driver1.POV.DOWN.whenPressed(commands.climber.Unsafe_Undeploy(self.robot))

		self.driver1.RIGHT_BUMPER.whenPressed(
			commands.climber.Safe_Wind(
				self.robot,
		 		self.driver1.RIGHT_BUMPER,
		 	)
		)
		self.driver1.LEFT_BUMPER.whenPressed(
		 	commands.climber.Safe_Unwind(
		 		self.robot,
		 		self.driver1.LEFT_BUMPER,
		 	)
		)

		self.driver1.RIGHT_BUMPER.whenReleased(commands.climber.Stop( self.robot ))
		self.driver1.LEFT_BUMPER.whenReleased(commands.climber.Stop( self.robot ))

		# slow down drive while climber is up (to 32%)

		self.jenny = .8675_309
		self.driver1.POV.UP.whenPressed(lambda: self.driver1.LEFT_JOY_Y.setModifier(lambda x: x * self.jenny ** 1))
		self.driver1.POV.UP.whenPressed(lambda: self.driver1.RIGHT_JOY_X.setModifier(lambda x: x * self.jenny ** 1))

		self.driver1.POV.DOWN.whenPressed(lambda: self.driver1.LEFT_JOY_Y.setModifier(None))
		self.driver1.POV.DOWN.whenPressed(lambda: self.driver1.RIGHT_JOY_X.setModifier(None))

		## Turret ##


	def log( self ):
		pass
