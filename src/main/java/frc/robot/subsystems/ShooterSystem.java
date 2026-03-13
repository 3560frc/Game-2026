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
  PIDMotor storageRoller;
  boolean storageOn = false;

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

    SlotConfigs configs2 = new SlotConfigs();
    configs2.kP = Constants.StorageRoller.kP;
    configs2.kI = Constants.StorageRoller.kI;
    configs2.kD = Constants.StorageRoller.kD;

    MotionMagicConfigs mmconfigs2 = new MotionMagicConfigs();
    mmconfigs2.MotionMagicAcceleration =
    Constants.StorageRoller.MAX_ACCELERATION_RPSPS;
    mmconfigs2.MotionMagicCruiseVelocity =
    Constants.StorageRoller.MAX_VELOCITY_RPS;

    storageRoller = PIDMotor.init(Constants.StorageRoller.MOTOR_ID, configs2,
    mmconfigs2, 1.0, Constants.StorageRoller.DIRECTION);

    shooterMotor2.disabled = true;
    storageRoller.disabled = true;
  }

  @Override
  public void periodic() {
    shooterMotor1.update();
    shooterMotor2.update();
    // storageRoller.update();
  }

  // public Command toggleStorage() {
  // return run(() -> {
  // this.storageOn = !this.storageOn;
  // if (this.storageOn) {
  // storageRoller.setVelocity(Constants.StorageRoller.VELOCITY_RPS);
  // } else {
  // storageRoller.setVelocity(0);
  // }
  // });
  // }

  public Command setState(boolean enabled) {
    return run(() -> {
      if (enabled) {
        // final int d = 210;
        // final double RPM2 = 310 * Math.sqrt(Math.pow(d, 2) / (1.73 * d - 77));
        // final double RPM1 = 1.78 * RPM2 / 2;
        // shooterMotor1.setVelocity(RPM1 / 60);
        // shooterMotor2.setVelocity(RPM2 / 60);

        shooterMotor1.setVelocity(Constants.ShooterRoller.VELOCITY_RPS);
        shooterMotor2.setVelocity(Constants.ShooterTop.VELOCITY_RPS);
      } else {
        shooterMotor1.setVelocity(0);
        shooterMotor2.setVelocity(0);
      }
    });
  }
}