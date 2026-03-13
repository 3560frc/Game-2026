// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.text.NumberFormat.Style;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterTop;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.IntakeSystem.IntakeDirection;
import frc.robot.subsystems.ShooterSystem;
// Jittering when goes fast
// Toggles don't work
// Climb doesn't work
import frc.robot.subsystems.VisionSystem;

public class RobotContainer {
        private double MaxSpeed = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
        private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                         // second
                                                                                         // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10%
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
        // private final ClimberSystem climberSystem = new ClimberSystem();
        private final IntakeSystem intakeSystem = new IntakeSystem();
        private final ShooterSystem shooterSystem = new ShooterSystem();
        private final VisionSystem vision;

        public RobotContainer() {
                vision = new VisionSystem(drivetrain);
                configureBindings();
                registerCommands();
                System.out.println(MaxSpeed);
        }

        // Pathplanner needs this to know how to call stuff
        private void registerCommands() {
                // NamedCommands.registerCommand("Shoot", ShooterSystem);
                // Intake
                // NamedCommands.registerCommand("enable_intake_rollers",
                // intakeSystem.setIntakeRollerEnabled(true));
                // NamedCommands.registerCommand("disable_intake_rollers",
                // intakeSystem.setIntakeRollerEnabled(false));
                // NamedCommands.registerCommand("extend_intake",
                // intakeSystem.setIntakeExtended(true));
                // NamedCommands.registerCommand("retract_intake",
                // intakeSystem.setIntakeExtended(false));

                // // Shooter
                // NamedCommands.registerCommand("enable_shooter_rollers",
                // shooterSystem.setState(true));
                // NamedCommands.registerCommand("disable_shooter_rollers",
                // shooterSystem.setState(false));

                // // Climber
                // NamedCommands.registerCommand("extend_climber",
                // climberSystem.setClimbExtended(true));
                // NamedCommands.registerCommand("retract_climber",
                // climberSystem.setClimbExtended(false));

                NamedCommands.registerCommand("Shoot", shooterSystem.setState(true));
        }

        private void configureBindings() {
                // inverted these
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(joystick.getLeftY() *
                                                                MaxSpeed) // Drive
                                                .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with
                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                // counterclockwise
                                // with
                                // negative
                                // X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // joystick.a().onTrue(shooterSystem.toggleStorage());
                // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(joystick.getRightY(), joystick.getRightX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                joystick.x().onTrue(Commands.runOnce(() -> intakeSystem.toggleIntakeExtended()));

                joystick.leftTrigger()
                                .whileTrue(intakeSystem.setIntakeRollerEnabled(true, IntakeDirection.REVERSE))
                                .whileFalse(intakeSystem.setIntakeRollerEnabled(false, IntakeDirection.STOP));

                // Shooter Buttons
                joystick.rightTrigger()
                                .whileTrue(shooterSystem.setState(true))
                                .whileFalse(shooterSystem.setState(false));

                joystick.povDown().onTrue(Commands.runOnce(() -> {
                        shooterSystem.reverseDirection = !shooterSystem.reverseDirection;
                }));

                joystick.povLeft().onTrue(Commands.runOnce(() -> {
                        shooterSystem.speedScale -= 0.05;
                }));

                joystick.povRight().onTrue(Commands.runOnce(() -> {
                        shooterSystem.speedScale += 0.05;
                }));

                joystick.b().onTrue(Commands.runOnce(() -> {
                        shooterSystem.toggleTop();
                }));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                // PathPlannerAuto auto = new PathPlannerAuto("BackShoot");
                return null;
                // return vision.autoCommand();
        }
}
