// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class RobotContainer {
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

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
  // private final StorageSystem storageSystem = new StorageSystem();

  // SG: see https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/RobotContainer.java
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    registerCommands();

    // Warmup PathPlanner to avoid Java pauses
    autoChooser = AutoBuilder.buildAutoChooser("BackShots");
    SmartDashboard.putData("Auto Mode", autoChooser);
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    SignalLogger.setPath("/media/sda1/ctre-logs/");
  }

  // SG: Pathplanner needs this to know how to call stuff
  private void registerCommands() {
    NamedCommands.registerCommand(
        "PreShoot",
        Commands.runOnce(() -> {
          shooterSystem.toggleShooter();
        }));
    NamedCommands.registerCommand(
        "Shoot",
        Commands.runOnce(() -> {
          shooterSystem.toggleFeed();
        }));
    NamedCommands.registerCommand(
        "StopShoot",
        Commands.runOnce(() -> {
          shooterSystem.toggleShooter();
          shooterSystem.toggleFeed();
        }));
    NamedCommands.registerCommand(
        "ToggleIntake",
        Commands.runOnce(() -> {
          // storageSystem.toggleStorage();
          intakeSystem.toggleIntakeRollers();
        }));
  }

  // press button to correct pose
  private void resetPoseOnButton(Trigger trigger, Pose2d pose) {
    trigger.onTrue(Commands.runOnce(() -> drivetrain.resetPose(pose)));
  }
  
  Pose2d HubCenter = new Pose2d(4.6304, 4.035, Rotation2d.fromDegrees(0));
  Pose2d Outpost = new Pose2d(0.4093, 0.3605, Rotation2d.fromDegrees(0));
  // Not sure
  Pose2d RightTrench = new Pose2d(4.630371794871795, 7.61648717948718, Rotation2d.fromDegrees(0));
  Pose2d LeftTrench = new Pose2d(4.630371794871795, 0.36048717948718034, Rotation2d.fromDegrees(0));

  private Pose2d TrackingPoint = HubCenter;

  class PointTrackData {
    double distance;
    Rotation2d targetRotation;
    double shooterVelocity;
  }

  // SG: aimbot
  private PointTrackData trackPointData(Pose2d point) {
    Pose2d absolutePose = drivetrain.getState().Pose;
    double deltax = absolutePose.getX() - point.getX();
    double deltay = absolutePose.getY() - point.getY();
    double d = Math.sqrt(Math.pow(deltax, 2) + Math.pow(deltay, 2));
    double angleToTarget = Math.atan2(deltax, deltay);

    // JC: calculate required RPM to shoot at angle
    final double RPM = 310 * Math.sqrt(Math.pow(d, 2) / (1.73 * d - 77));

    PointTrackData data = new PointTrackData();
    data.distance = d;
    data.shooterVelocity = RPM;
    data.targetRotation = Rotation2d.fromRadians(angleToTarget);
    return data;
  }

  private void configureDriveBindings() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    DriveController.rightTrigger()
        .onTrue(Commands.runOnce(() -> intakeSystem.toggleIntakeRollers()))
        .onFalse(Commands.runOnce(() -> intakeSystem.toggleIntakeRollers()));

    // SG: for storage rollers we don't use anymore
    // DriveController.leftTrigger()
    // .onTrue(Commands.runOnce(() -> storageSystem.toggleStorage()))
    // .onFalse(Commands.runOnce(() -> storageSystem.toggleStorage()));

    DriveController.a().onTrue(Commands.runOnce(() -> intakeSystem.setIntakeHingeDown()));
    DriveController.y().onTrue(Commands.runOnce(() -> intakeSystem.setIntakeHingeUp()));
    DriveController.b().onTrue(Commands.runOnce(() -> intakeSystem.stopIntakeHinge()));

    DriveController.povDown().onTrue(Commands.runOnce(() -> {
      shooterSystem.reverseDirection = !shooterSystem.reverseDirection;
    }));
  }

  private void configureBindings() {
    // SG: SysID bindings -> use to find PIDs for swerve drive
    UtilsController.x().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    UtilsController.y().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    UtilsController.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    UtilsController.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    UtilsController.povRight().onTrue(Commands.runOnce(SignalLogger::start));
    UtilsController.povLeft().onTrue(Commands.runOnce(SignalLogger::stop));

    configureDriveBindings();

    SlewRateLimiter y = new SlewRateLimiter(0.85);
    SlewRateLimiter x = new SlewRateLimiter(0.85);
    // SlewRateLimiter r = new SlewRateLimiter(0.85);

    // SG: if backwards is forwards and forwards is backwards flip the controls
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              return drive
                  .withVelocityX(-y.calculate(DriveController.getLeftY()) * MaxSpeed)
                  .withVelocityY(-x.calculate(DriveController.getLeftX()) * MaxSpeed)
                  .withRotationalRate(DriveController.getRightX() * MaxAngularRate);
            }));

    // drivetrain.setDefaultCommand(
    // drivetrain.applyRequest(
    // () -> {
    // double lefty = DriveController.getLeftY();
    // double leftx = DriveController.getLeftX();
    // double rightx = DriveController.getRightX();

    // System.out.println(leftx);

    // return drive.withVelocityX(-StrictMath.pow(lefty, 3) * MaxSpeed)
    // .withVelocityY(-StrictMath.pow(leftx, 3)
    // * MaxSpeed)
    // .withRotationalRate(-StrictMath.pow(rightx, 3)
    // * MaxAngularRate);
    // }));

    // UtilsController.rightBumper().toggleOnTrue(Commands.runOnce(() -> {
    // PointTrackData data = trackPointData(TrackingPoint);
    // shooterSystem.setShooterVelocity(data.shooterVelocity);
    // shooterSystem.toggleShooter();

    // drivetrain.setControl(face);

    // drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
    // double xvalue = Math.pow(DriveController.getLeftX(), 2);
    // double yvalue = Math.pow(DriveController.getLeftY(), 2);

    // return face.withVelocityX(-xvalue * MaxSpeed)
    // .withVelocityY(-yvalue * MaxSpeed)
    // .withTargetDirection(data.targetRotation);
    // }));
    // })).toggleOnFalse(Commands.runOnce(() -> {
    // shooterSystem.resetShooterVelocity();
    // shooterSystem.toggleShooter();

    // drivetrain.setControl(drive);

    // drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
    // double xvalue = Math.pow(DriveController.getLeftX(), 2);
    // double yvalue = Math.pow(DriveController.getLeftY(), 2);
    // double rotation = Math.pow(DriveController.getRightX(), 2);

    // return drive.withVelocityX(-xvalue * MaxSpeed)
    // .withVelocityY(-yvalue * MaxSpeed)
    // .withRotationalRate(rotation * MaxAngularRate);
    // }));
    // }));

    // Ordinary Shooting
    UtilsController.rightTrigger()
        // .whileTrue(Commands.run(() -> shooterSystem
        // .activateShooterWithSpeed(DriveController.getRightTriggerAxis())));
        .onTrue(Commands.runOnce(() -> shooterSystem.toggleShooter()))
        .onFalse(Commands.runOnce(() -> shooterSystem.toggleShooter()));

    UtilsController.leftTrigger()
        // .whileTrue(Commands.run(() -> shooterSystem
        // .activateFeedWithSpeed(DriveController.getLeftTriggerAxis())));
        .onTrue(Commands.runOnce(() -> shooterSystem.toggleFeed()))
        .onFalse(Commands.runOnce(() -> shooterSystem.toggleFeed()));

    // Update Field Position
    UtilsController.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    // resetPoseOnButton(UtilsController.povDown(), Outpost);
    // resetPoseOnButton(UtilsController.povRight(), RightTrench);
    // resetPoseOnButton(UtilsController.povLeft(), LeftTrench);

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    // chooses the auto from GUI
    return autoChooser.getSelected();
  }
}
