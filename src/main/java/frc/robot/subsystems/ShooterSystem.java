package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ShooterSystem extends SubsystemBase {
  PIDMotor shooterMotor;
  public boolean reverseDirection;
  public boolean shooting = true;
  private double targetVelocity = 0;

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

    resetShooterVelocity();
  }

  @Override
  public void periodic() {
    shooterMotor.update();
  }

  public void setShooterVelocity(double velocity) {
    this.targetVelocity = velocity;
  }

  public void resetShooterVelocity() {
    this.targetVelocity = Constants.ShooterRoller.VELOCITY_RPS;
  }

  public void toggleShooter() {
    this.shooting = !this.shooting;

    if (this.shooting) {
      if (!reverseDirection) {
        shooterMotor.setVelocity(this.targetVelocity);
      } else {
        shooterMotor.setVelocity(-this.targetVelocity);
      }
    } else {
      shooterMotor.setVelocity(0);
    }
  }
}