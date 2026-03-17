package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ShooterSystem extends SubsystemBase {
  PIDMotor shooterMotor;
  public boolean reverseDirection;
  public float speedScale = 1;
  public boolean shooting = true;

  public ShooterSystem() {
    SlotConfigs configs = new SlotConfigs();
    configs.kP = Constants.ShooterRoller.kP;
    configs.kI = Constants.ShooterRoller.kI;
    configs.kD = Constants.ShooterRoller.kD;

    MotionMagicConfigs mmconfigs = new MotionMagicConfigs();
    mmconfigs.MotionMagicAcceleration = Constants.ShooterRoller.MAX_ACCELERATION_RPSPS;
    mmconfigs.MotionMagicCruiseVelocity = Constants.ShooterRoller.MAX_VELOCITY_RPS;

    shooterMotor = PIDMotor.init(Constants.ShooterRoller.MOTOR_ID, configs, mmconfigs, 1.0,
        Constants.ShooterRoller.DIRECTION);
  }

  @Override
  public void periodic() {
    shooterMotor.update();
  }

  public void toggleShooter() {
    this.shooting = !this.shooting;

    if (this.shooting) {
      // final int d = 210;
      // final double RPM2 = 310 * Math.sqrt(Math.pow(d, 2) / (1.73 * d - 77));
      // final double RPM1 = 1.78 * RPM2 / 2;
      // shooterMotor1.setVelocity(RPM1 / 60);
      // shooterMotor2.setVelocity(RPM2 / 60);

      if (!reverseDirection) {
        shooterMotor.setVelocity(Constants.ShooterRoller.VELOCITY_RPS * speedScale);
      } else {
        shooterMotor.setVelocity(-Constants.ShooterRoller.VELOCITY_RPS * speedScale);
      }
    } else {
      shooterMotor.setVelocity(0);
    }
  }
}