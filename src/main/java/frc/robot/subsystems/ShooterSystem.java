package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ShooterSystem extends SubsystemBase {
  PIDMotor shooterMotor2;
  PIDMotor shooterMotor1;
  public boolean reverseDirection;
  public float speedScale = 1;
  private boolean topEnabled = false;

  public ShooterSystem() {
    SlotConfigs configs = new SlotConfigs();
    configs.kP = Constants.ShooterTop.kP;
    configs.kI = Constants.ShooterTop.kI;
    configs.kD = Constants.ShooterTop.kD;

    MotionMagicConfigs mmconfigs = new MotionMagicConfigs();
    mmconfigs.MotionMagicAcceleration = Constants.ShooterTop.MAX_ACCELERATION_RPSPS;
    mmconfigs.MotionMagicCruiseVelocity = Constants.ShooterTop.MAX_VELOCITY_RPS;

    shooterMotor2 = PIDMotor.init(Constants.ShooterTop.MOTOR_ID, configs, mmconfigs, 1.0,
        Constants.ShooterTop.DIRECTION);

    SlotConfigs configs1 = new SlotConfigs();
    configs1.kP = Constants.ShooterRoller.kP;
    configs1.kI = Constants.ShooterRoller.kI;
    configs1.kD = Constants.ShooterRoller.kD;

    MotionMagicConfigs mmconfigs1 = new MotionMagicConfigs();
    mmconfigs1.MotionMagicAcceleration = Constants.ShooterRoller.MAX_ACCELERATION_RPSPS;
    mmconfigs1.MotionMagicCruiseVelocity = Constants.ShooterRoller.MAX_VELOCITY_RPS;

    shooterMotor1 = PIDMotor.init(Constants.ShooterRoller.MOTOR_ID, configs1, mmconfigs1, 1.0,
        Constants.ShooterRoller.DIRECTION);

    // shooterMotor2.disabled = true;
  }

  @Override
  public void periodic() {
    shooterMotor1.update();
    shooterMotor2.update();
  }

  public void toggleTop() {
    topEnabled = !topEnabled;

    System.out.println(topEnabled);

    if (topEnabled) {
      if (!reverseDirection) {
        shooterMotor2.setVelocity(Constants.ShooterTop.VELOCITY_RPS * speedScale);
      } else {
        shooterMotor2.setVelocity(-Constants.ShooterTop.VELOCITY_RPS * speedScale);
      }
    } else {
      shooterMotor2.setVelocity(0);
    }
  }

  public Command setState(boolean enabled) {
    return run(() -> {
      if (enabled) {
        // final int d = 210;
        // final double RPM2 = 310 * Math.sqrt(Math.pow(d, 2) / (1.73 * d - 77));
        // final double RPM1 = 1.78 * RPM2 / 2;
        // shooterMotor1.setVelocity(RPM1 / 60);
        // shooterMotor2.setVelocity(RPM2 / 60);

        if (!reverseDirection) {
          shooterMotor1.setVelocity(Constants.ShooterRoller.VELOCITY_RPS * speedScale);
        } else {
          shooterMotor1.setVelocity(-Constants.ShooterRoller.VELOCITY_RPS * speedScale);
        }
      } else {
        shooterMotor1.setVelocity(0);
      }
    });
  }
}