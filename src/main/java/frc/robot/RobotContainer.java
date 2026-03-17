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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.IntakeSystem.IntakeDirection;
import frc.robot.subsystems.ShooterSystem;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                      // top
        private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per
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

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // SubSystems
        private final ClimberSystem climberSystem = new ClimberSystem();
        private final IntakeSystem intakeSystem = new IntakeSystem();
        private final ShooterSystem shooterSystem = new ShooterSystem();

        /* Path follower */
        private final SendableChooser<Command> autoChooser;

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

        Pose2d HubCenter = new Pose2d(4.6304, 4.035, null);
        Pose2d Outpost = new Pose2d(0.4093, 0.3605, null);
        // Not sure
        Pose2d RightTrench = new Pose2d(4.630371794871795, 7.61648717948718, null);
        Pose2d LeftTrench = new Pose2d(4.630371794871795, 0.36048717948718034, null);

        // press button to correct pose when you push push the balls in the corner for
        // aimbot
        private void resetPoseOutpost() {
                Pose2d pose = drivetrain.getState().Pose;
                joystick.leftBumper().onTrue(
                                Commands.runOnce(() -> drivetrain.resetPose(
                                                new Pose2d(Outpost.getX(), Outpost.getY(), pose.getRotation()))));
        }

        private void trackCenter() {
                Pose2d pose = drivetrain.getState().Pose;
        }

        private void configureBindings() {
                double xvalue = Math.pow(joystick.getLeftY(), 2);
                double yvalue = Math.pow(joystick.getLeftX(), 2);
                double rotation = Math.pow(joystick.getRightX(), 2);

                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(xvalue * MaxSpeed)
                                                .withVelocityY(yvalue * MaxSpeed)
                                                .withRotationalRate(-rotation * MaxAngularRate)));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                joystick.y().onTrue(Commands.runOnce(() -> climberSystem.toggleClimbExtended()));

                joystick.x().onTrue(Commands.runOnce(() -> intakeSystem.toggleIntakeExtended()));

                joystick.leftTrigger()
                                .whileTrue(intakeSystem.setIntakeRollerEnabled(true, IntakeDirection.REVERSE))
                                .whileFalse(intakeSystem.setIntakeRollerEnabled(false, IntakeDirection.STOP));

                // Shooter Buttons
                joystick.rightTrigger()
                                .onTrue(Commands.runOnce(() -> shooterSystem.toggleShooter()))
                                .onFalse(Commands.runOnce(() -> shooterSystem.toggleShooter()));

                joystick.povDown().onTrue(Commands.runOnce(() -> {
                        shooterSystem.reverseDirection = !shooterSystem.reverseDirection;
                }));

                joystick.povLeft().onTrue(Commands.runOnce(() -> {
                        shooterSystem.speedScale -= 0.05;
                }));

                joystick.povRight().onTrue(Commands.runOnce(() -> {
                        shooterSystem.speedScale += 0.05;
                }));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                // chooses the auto from GUI
                return autoChooser.getSelected();
        }
}
