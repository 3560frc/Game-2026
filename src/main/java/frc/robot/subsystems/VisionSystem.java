package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSystem extends SubsystemBase {
    private SwerveDrivePoseEstimator m_poseEstimator;
    private Pigeon2 m_gyro;

    // IDK about the rest of these params
    public VisionSystem() {
        m_poseEstimator = new SwerveDrivePoseEstimator(null, m_gyro.getRotation2d(), null, null);
    }

    public Command updateRobotPose() {
        return run(() -> {
            m_poseEstimator.update(m_gyro.getRotation2d(), null);

            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

            if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                m_poseEstimator.addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds
                );
            }
        });
    }
}
