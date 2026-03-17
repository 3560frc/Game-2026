// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.IntakeSystem.IntakeDirection;
import frc.robot.subsystems.ShooterSystem;

public class RobotContainer {
        private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
        private final double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                               // second
                                                                                               // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10%
                                                                                                   // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective); // Use open-loop control
                                                                                              // for drive
                                                                                              // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentricFacingAngle face = new SwerveRequest.RobotCentricFacingAngle();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController DriveController = new CommandXboxController(1);
        private final CommandXboxController UtilsController = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // SubSystems
        private final IntakeSystem intakeSystem = new IntakeSystem();
        private final ShooterSystem shooterSystem = new ShooterSystem();

        /* Path follower */
        private final SendableChooser<Command> autoChooser;

        Pose2d HubCenter = new Pose2d(4.6304, 4.035, Rotation2d.fromDegrees(0));
        Pose2d Outpost = new Pose2d(0.4093, 0.3605, Rotation2d.fromDegrees(0));
        // Not sure
        Pose2d RightTrench = new Pose2d(4.630371794871795, 7.61648717948718, Rotation2d.fromDegrees(0));
        Pose2d LeftTrench = new Pose2d(4.630371794871795, 0.36048717948718034, Rotation2d.fromDegrees(0));

        private Pose2d TrackingPoint = HubCenter;

        public RobotContainer() {
                configureBindings();
                registerCommands();

                // Warmup PathPlanner to avoid Java pauses
                autoChooser = AutoBuilder.buildAutoChooser("BackShots");
                SmartDashboard.putData("Auto Mode", autoChooser);
                CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        }

        // Pathplanner needs this to know how to call stuff
        private void registerCommands() {
                NamedCommands.registerCommand(
                                "Shoot",
                                Commands.runOnce(() -> shooterSystem.toggleShooter()));
                NamedCommands.registerCommand(
                                "StopShoot",
                                Commands.runOnce(() -> shooterSystem.toggleShooter()));
                NamedCommands.registerCommand(
                                "ToggleIntake",
                                Commands.runOnce(() -> intakeSystem.toggleIntakeExtended()));
        }

        // press button to correct pose
        private void resetPoseOnButton(Trigger trigger, Pose2d pose) {
                trigger.onTrue(Commands.runOnce(() -> drivetrain.resetPose(pose)));
        }

        class PointTrackData {
                double distance;
                Rotation2d targetRotation;
                double shooterVelocity;
        }

        // aimbot
        private PointTrackData trackPointData(Pose2d point) {
                Pose2d absolutePose = drivetrain.getState().Pose;
                double deltax = absolutePose.getX() - point.getX();
                double deltay = absolutePose.getY() - point.getY();
                double d = Math.sqrt(Math.pow(deltax, 2) + Math.pow(deltay, 2));
                double angleToTarget = Math.atan2(deltax, deltay);

                final double RPM = 310 * Math.sqrt(Math.pow(d, 2) / (1.73 * d - 77));

                PointTrackData data = new PointTrackData();
                data.distance = d;
                data.shooterVelocity = RPM;
                data.targetRotation = Rotation2d.fromRadians(angleToTarget);
                return data;
        }

        private void configureBindings() {
                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // DriveController.back().and(DriveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // DriveController.back().and(DriveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // DriveController.start().and(DriveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // DriveController.start().and(DriveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                DriveController.leftTrigger()
                                .whileTrue(intakeSystem.setIntakeRollerEnabled(true, IntakeDirection.REVERSE))
                                .whileFalse(intakeSystem.setIntakeRollerEnabled(false, IntakeDirection.STOP));

                // Shooter Buttons
                DriveController.rightTrigger()
                                .onTrue(Commands.runOnce(() -> shooterSystem.toggleShooter()))
                                .onFalse(Commands.runOnce(() -> shooterSystem.toggleShooter()));

                DriveController.povDown().onTrue(Commands.runOnce(() -> {
                        shooterSystem.reverseDirection = !shooterSystem.reverseDirection;
                }));

                // enable tracking
                DriveController.rightBumper().onTrue(Commands.runOnce(() -> {
                        PointTrackData data = trackPointData(TrackingPoint);
                        shooterSystem.setShooterVelocity(data.shooterVelocity);
                        shooterSystem.toggleShooter();

                        drivetrain.setControl(face);

                        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
                                double xvalue = Math.pow(DriveController.getLeftX(), 2);
                                double yvalue = Math.pow(DriveController.getLeftY(), 2);

                                return face.withVelocityX(xvalue * MaxSpeed)
                                                .withVelocityY(yvalue * MaxSpeed)
                                                .withTargetDirection(data.targetRotation);
                        }));
                })).onFalse(Commands.runOnce(() -> {
                        shooterSystem.resetShooterVelocity();
                        shooterSystem.toggleShooter();

                        drivetrain.setControl(drive);

                        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
                                double xvalue = Math.pow(DriveController.getLeftX(), 2);
                                double yvalue = Math.pow(DriveController.getLeftY(), 2);
                                double rotation = Math.pow(DriveController.getRightX(), 2);

                                return drive.withVelocityX(xvalue * MaxSpeed)
                                                .withVelocityY(yvalue * MaxSpeed)
                                                .withRotationalRate(-rotation * MaxAngularRate);
                        }));
                }));

                // Utils
                DriveController.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
                resetPoseOnButton(UtilsController.povDown(), Outpost);
                resetPoseOnButton(UtilsController.povRight(), RightTrench);
                resetPoseOnButton(UtilsController.povLeft(), LeftTrench);

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                // chooses the auto from GUI
                return autoChooser.getSelected();
        }
}
