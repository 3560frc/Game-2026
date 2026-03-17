package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSystem extends SubsystemBase {
  private SwerveDrivePoseEstimator m_poseEstimator;
  private Pigeon2 m_gyro;
  private CommandSwerveDrivetrain m_drivetrain;
  private SwerveRequest.ApplyRobotSpeeds m_drive;

  private ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveDriveState drivetrain_state = m_drivetrain.getState();
    return drivetrain_state.Speeds;
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    m_drivetrain.setControl(m_drive.withSpeeds(speeds));
  }

  public VisionSystem(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_gyro = drivetrain.getPigeon2();
    m_drive = new SwerveRequest.ApplyRobotSpeeds();

    SwerveDriveState drivetrain_state = m_drivetrain.getState();
    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_drivetrain.getKinematics(),
        m_gyro.getRotation2d(),
        drivetrain_state.ModulePositions,
        drivetrain_state.Pose);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          () -> m_poseEstimator.getEstimatedPosition(),
          (pose) -> m_poseEstimator.resetPose(pose),
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    SwerveDriveState drivetrain_state = m_drivetrain.getState();
    m_poseEstimator.update(m_gyro.getRotation2d(), drivetrain_state.ModulePositions);

    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    if (limelightMeasurement.tagCount >= 2) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      m_poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds);
    }
  }

  public Command autoCommand() {
    return null;
  }
}
