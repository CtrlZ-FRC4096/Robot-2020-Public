"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2020-2021
Code for robot "Krono-Z"
contact@team4096.org

Billy sez priority is:
1. Stationary 3 ball
2. Same side 5 ball
3. Middle 5 ball
4. 8 ball
5. Opposite side 5 ball

"""

# This is to help vscode
from math import pi
from typing import List, TYPE_CHECKING
from commands2 import InstantCommand
from wpilib.controller import PIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib.controller import RamseteController
from wpimath.trajectory import TrajectoryGenerator, Trajectory, TrajectoryUtil
from pathlib import Path
from typing import Union

if TYPE_CHECKING:
	from robot import Robot

from commands2 import SequentialCommandGroup, ParallelRaceGroup, ParallelCommandGroup, WaitCommand, RamseteCommand

from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.controller import SimpleMotorFeedforward, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrajectoryConfig
from const import KP_DRIVE_VEL, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, MAX_SPEED_METERS_PER_SECOND, KS_VOLTS, KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER, RAMSETE_B, RAMSETE_ZETA
import commands.drivetrain
import commands.shooter
import commands.hood
import commands.indexer
import commands.intake
# import paths


def getConfig(robot: "Robot"):
	# (self, kS: volts, kV: volt_seconds_per_meter, kA: volt_seconds_squared_per_meter = 0.0)
	feed_forward = SimpleMotorFeedforwardMeters(
		KS_VOLTS,
		KV_VOLT_SECONDS_PER_METER,
		KA_VOLT_SECONDS_SQUARED_PER_METER
	)

	# DifferentialDriveVoltageConstraint​(SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, double maxVoltage)

	autoVoltageConstraint = DifferentialDriveVoltageConstraint(
		feed_forward,
		robot.drive.kinematics,
		10
	)

	config = TrajectoryConfig(MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
	config.setKinematics(robot.drive.kinematics)
	config.addConstraint(autoVoltageConstraint)
	return config


def getSlalomPathCommand(robot: "Robot"):
	in2m = 1 / 39.3701

	initialPosition = Pose2d(-19 * in2m, -33 * in2m, Rotation2d.fromDegrees(27))

	movements = [
		Translation2d(0 * in2m, -5 * in2m),
		Translation2d(5 * in2m, 5 * in2m),
		Translation2d(60 * in2m, 30* in2m),
		Translation2d(190 * in2m, 20 * in2m),
		Translation2d(206 * in2m, -33 * in2m),
		#Translation2d(294 * in2m, 33 * in2m),
	]

	finalPosition = Pose2d(274 * in2m, 33 * in2m, Rotation2d(pi / 2))
	config = getConfig(robot)

	exampleTrajectory = TrajectoryGenerator.generateTrajectory(
		initialPosition, movements, finalPosition, config,
	 )

	return getPathCommand(robot, exampleTrajectory)


def getPathCommand(robot: "Robot", trajectory: Union[Trajectory, str]):
	if not isinstance(trajectory, Trajectory):
		#trajectory = TrajectoryUtil.fromPathweaverJson(str(Path(__file__).parent / "paths" / "output" / trajectory))
		trajectory = TrajectoryUtil.fromPathweaverJson(str(Path(__file__).parent.parent / "paths" / "output" / trajectory))

	config = getConfig(robot)

	# initialPosition = Pose2d(0,0, Rotation2d(0))

	# movements = [Translation2d(1,1), Translation2d(2, -1)]
	# # movements = []
	# # finalPosition = Pose2d(1,1, Rotation2d(pi / 2))
	# finalPosition = Pose2d(3,0, Rotation2d(0))
	# exampleTrajectory = TrajectoryGenerator.generateTrajectory(
		# initialPosition, movements, finalPosition, config,
	# )

	ramseteCommand = RamseteCommand(
		trajectory,
		robot.drive.getPose,
		RamseteController(RAMSETE_B, RAMSETE_ZETA),
		SimpleMotorFeedforwardMeters(KS_VOLTS, KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER),
		robot.drive.kinematics,
		robot.drive.get_wheel_speeds,
		PIDController(KP_DRIVE_VEL,0,0),
		PIDController(KP_DRIVE_VEL,0,0),
		robot.drive.tankDriveVolts,
		[robot.drive],
	)

	return (
		InstantCommand(lambda: robot.drive.gyro.reset())
			.andThen(InstantCommand(lambda: robot.drive.reset_odometry(trajectory.initialPose())))
			.andThen(ramseteCommand)
			.andThen(lambda: robot.drive.tankDriveVolts(0,0))
	)



class Do_Nothing(SequentialCommandGroup):
	'''
	This autonomous mode does... wait for it... nothing.
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(commands.drivetrain.Stop(self.robot))


class Cross_Baseline(SequentialCommandGroup):
	'''
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(commands.drivetrain.Drive_Distance(self.robot, 5.0))

# class First_Ball(SequentialCommandGroup):

# 	def __init(self, robot: "Robot"):
# 		super().__init__()

# 		self.robot = robot

# 		self.addCommands(command.drivetrain.Drive_Distance(self.robot, 3.0))
# 		self.addCommands(command.drivetrain.Rotate_To_Angle(self.robot, ? ))


class Behind_10_Foot(ParallelCommandGroup):
	'''
	Describe this auto mode here
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(
			ParallelCommandGroup(
				commands.shooter.Run_Velocity(self.robot, 70_000),
				commands.hood.Set_Hood_Position(self.robot, 25),
				WaitCommand(5).andThen(commands.indexer.Feed(self.robot))
			)
		)

class Six_Ball(ParallelCommandGroup):
	'''
	Describe this auto mode here
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(
			SequentialCommandGroup(
				commands.intake.Down(self.robot),
				ParallelCommandGroup(
					commands.shooter.Run_Velocity(self.robot, 67_500),
					commands.hood.Set_Hood_Position(self.robot, 24.5),
					SequentialCommandGroup(
						WaitCommand(2),
						ParallelRaceGroup(
							commands.indexer.Feed(self.robot),
							WaitCommand(2),
						),
						commands.indexer.Stop(self.robot),
						ParallelRaceGroup(
							commands.intake.In(self.robot),
							commands.indexer.Out(self.robot),
							Run_Pathweaver_Path(self.robot, "Ball_Auto1.wpilib.json"),
							WaitCommand(3.5),
						),
						ParallelRaceGroup(
							Run_Pathweaver_Path(self.robot, "Ball_Auto2.wpilib.json"),
							commands.intake.In(self.robot),
							WaitCommand(2.5),
						),
						# commands.intake.Up(self.robot),
						ParallelCommandGroup(
							commands.intake.In(self.robot),
							# InstantCommand(lambda: self.robot.intake.run(self.robot, .2)),
							commands.indexer.Feed(self.robot),
						),
					)
				)
			)
		)



		# self.addCommands(
		# 	ParallelCommandGroup(
		# 		commands.intake.Down(self.robot),
		# 		commands.shooter.Run_Velocity(self.robot, 70_000),
		# 		commands.hood.Set_Hood_Position(self.robot, 35),
		# 		WaitCommand(2).andThen(commands.indexer.Feed(self.robot)),
		# 		WaitCommand(5).andThen(commands.indexer.Stop(self.robot)),

		# 		WaitCommand(5).andThen(commands.intake.In(self.robot)),
		# 		WaitCommand(5).andThen(Run_Pathweaver_Path(self.robot, "Ball_Auto1.wpilib.json"))
		# 	)
		# )


class Shoot_3_Balls_From_B4_10(ParallelCommandGroup):
	'''
	Describe this auto mode here
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(
			ParallelCommandGroup(
				commands.shooter.Run_Velocity(self.robot, 70_000),
				commands.hood.Set_Hood_Position(self.robot, 35),
				WaitCommand(5).andThen(commands.indexer.Feed(self.robot))
			)
		)
class Front_10_Foot(SequentialCommandGroup):
	'''
	Describe this auto mode here
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(
			SequentialCommandGroup(
				Shoot_3_Balls_From_B4_10(self.robot),
				commands.drivetrain.Drive_With_Tank_Values(self.robot,0,0.5).withTimeout(5)
			)
		)
class Eleven_Foot(ParallelCommandGroup):
	'''
	Describe this auto mode here
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(
			ParallelCommandGroup(
				commands.shooter.Run_Velocity(self.robot, 66_000),
				commands.hood.Set_Hood_Position(self.robot, 26.5),
				WaitCommand(5).andThen(commands.indexer.Feed(self.robot))
			)
		)


class Same_Side_5_Ball(SequentialCommandGroup):
	'''
	Describe this auto mode here
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(commands.shooter.Shoot_N_Balls(self.robot, 3))


class Middle_5_Ball(SequentialCommandGroup):
	'''
	Describe this auto mode here
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(commands.shooter.Shoot_N_Balls(self.robot, 3))


class Eight_Ball(SequentialCommandGroup):
	'''
	Describe this auto mode here
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(commands.shooter.Shoot_N_Balls(self.robot, 3))


class Far_Side_5_Ball(SequentialCommandGroup):
	'''
	Describe this auto mode here
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(commands.shooter.Shoot_N_Balls(self.robot, 3))


class Run_Pathweaver_Path(ParallelCommandGroup):
	'''
	Galactic Search or other json path from Pathweaver
	'''
	def __init__(self, robot: "Robot", trajectory_filename: str):
		super().__init__()

		self.robot = robot

		# self.addCommands(commands.autonomous.Intake_Deploy_And_Run(self.robot))
		self.addCommands(commands.autonomous.getPathCommand(self.robot, trajectory_filename))


class Intake_Deploy_And_Run(SequentialCommandGroup):
	'''
	Does what it says
	'''
	def __init__(self, robot: "Robot"):
		super().__init__()

		self.robot = robot

		self.addCommands(commands.intake.Down(self.robot))
		self.addCommands(commands.intake.In(self.robot))