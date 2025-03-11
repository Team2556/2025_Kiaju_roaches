#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import button, cmd

import numpy as np
from commands2.sysid import SysIdRoutine

from constants import ClimbConstants
from generated.tuner_constants import TunerConstants
from constants import RobotDimensions, ElevatorConstants
from subsystems import (
    ElevatorSubsystem,
    coralSubsystem,
    limelightSubsystem,
    pneumaticSubsystem,
    # oneMotor,
    ultrasonic, #ClimbSubsystem
)
from telemetry import Telemetry
from robotUtils import controlAugment

from phoenix6 import swerve
from phoenix6.hardware import TalonFX
import wpilib
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Rotation2d, Translation2d, Transform2d, Pose2d, Rectangle2d
from wpimath.units import (
    rotationsToRadians,
    degrees,
    radians,
    degreesToRadians,
    radiansToDegrees,
    metersToInches,
    inchesToMeters,
)
import wpinet
import math
from commands.odometrySnap2Line import SnapToLineCommand

# from commands.gotoClosestPath import GotoClosestPath
# from commands.drive_one_motor import DriveOneMotorCommand
from commands.liftElevator import LiftElevatorCommand
# from commands import coralCommand
import networktables as nt
from networktables import util as ntutil



# from subsystems import algae


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        #TODO: FIX ME!!!! this needs to look at the assigned allinace and work out to 1 or -1
        self.invertBlueRedDrive = 1

        self.robotWidthBumpered = inchesToMeters(RobotDimensions.WIDTH_w_bumpers)
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.08)
            .with_rotational_deadband(
                self._max_angular_rate * 0.08
            )  # Add a 5% deadband on output
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY #OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive
        )
        """
        Control the drive motor using a velocity closed-loop request.
        The control output type is determined by SwerveModuleConstants.DriveMotorClosedLoopOutput
        """


        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY #.OPEN_LOOP_VOLTAGE
            )
        )

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)
        # This one's probably used for scoring stuff
        self._joystick2 = commands2.button.CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        self.coralENABLE = False
        if self.coralENABLE:
            self.coral_track = coralSubsystem.CoralTrack()
            self.default_coralTrackStop = self.coral_track.set_motor(0)
            self.TEMP_score_Right = self.coral_track.set_motor(0.18)
            self.TEMP_score_Left = self.coral_track.set_motor(-0.18)
            
        pneumaticENABLE = False
        if pneumaticENABLE:
            self.pneumaticsHub = pneumaticSubsystem.PneumaticSubsystem()
        self.ENABLE_ELEVATOR = False
        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -controlAugment.smooth(
                            self._joystick.getLeftY(), exponential_for_curving=3
                        )
                        * self._max_speed
                        * self.invertBlueRedDrive
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -controlAugment.smooth(
                            self._joystick.getLeftX(), exponential_for_curving=3
                        )
                        * self._max_speed
                        * self.invertBlueRedDrive
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -controlAugment.smooth(self._joystick.getRightX())
                        * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                    .with_center_of_rotation(
                        Translation2d(
                            x=self.robotWidthBumpered
                            * 0.2
                            * (
                                controlAugment.smooth(
                                    controlAugment.one_side_control_only(
                                        self._joystick.getRightY(), "Pos"
                                    )
                                )
                            ),
                            # want y translation to depend on direction of turn
                            y=math.copysign(1, self._joystick.getRightX()),
                        )
                        * self.robotWidthBumpered
                        * 0.2
                        * (
                            controlAugment.smooth(
                                controlAugment.one_side_control_only(
                                    self._joystick.getRightY(), "Pos"
                                )
                            )
                        )
                    )
                    # shift the center of rotation to opposite front corner, if the driver pulls down on the right stick in addition to the side.
                    # This should allow some nice defensive roll-off maneuvers
                )
            )
        )

        if self.coralENABLE:
            self.coral_track.setDefaultCommand(
                cmd.run(lambda: self.default_coralTrackStop, self.coral_track)
            )
            self._joystick.povRight().whileTrue(cmd.run(lambda: self.TEMP_score_Right, self.coral_track))
            self._joystick.povLeft().whileTrue(cmd.run(lambda: self.TEMP_score_Left, self.coral_track))

        # self.one_motor.setDefaultCommand(DriveOneMotorCommand(self.one_motor, self._joystick2))
        if self.ENABLE_ELEVATOR:
            self.elevator.setDefaultCommand(
                LiftElevatorCommand(self.elevator, self._joystick2)
            )
            (self._joystick2.start() & self._joystick2.a()).whileTrue(
                self._reset_zero_point_here
            )  # .onFalse(lambda: self._elevator_motors_break) #TODO: fix this to not crash :)
            # (self._joystick2.start() & self._joystick2.x()).whileTrue(lambda: self._reset_zero_point_here) #TODO: fix this to not crash :)

        # section vision related commands
                    
        self._joystick.x().onTrue(SnapToLineCommand(self.drivetrain))

        # endsection vision related commands

        # Coral Track controls
        # self.coral_track.setDefaultCommand(self.coral_command)

        # self._joystick2.x().whileTrue(
        #     commands2.cmd.run(
        #         lambda: self.coral_command.enable_discharge(), self.coral_track
        #     )
        # )

        # self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return commands2.cmd.print_("No autonomous command configured")
