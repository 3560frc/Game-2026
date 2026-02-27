package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ShooterSystem extends SubsystemBase {
  PIDMotor shooterMotor2 = PIDMotor.initVelocityOnly(
      Constants.ShooterRoller.MOTOR_ID,
      Constants.ShooterRoller.kP,
      Constants.ShooterRoller.kI,
      Constants.ShooterRoller.kD,
      Constants.ShooterRoller.DIRECTION);

  PIDMotor shooterMotor1 = PIDMotor.initVelocityOnly(
      Constants.ShooterTop.MOTOR_ID,
      Constants.ShooterTop.kP,
      Constants.ShooterTop.kI,
      Constants.ShooterTop.kD,
      Constants.ShooterTop.DIRECTION);

  @Override
  public void periodic() {
    shooterMotor1.update();
    shooterMotor2.update();
  }

  public Command setState(boolean enabled) {
    return run(() -> {
      if (enabled) {
        final int d = 210;
        final double RPM2 = 310 * Math.sqrt(Math.pow(d, 2) / (1.73 * d - 77));
        final double RPM1 = 1.78 * RPM2;
        shooterMotor1.setVelocity(RPM1 / 60);
        shooterMotor2.setVelocity(RPM2 / 60);
      } else {
        shooterMotor1.setVelocity(0);
        shooterMotor2.setVelocity(0);
      }
    });
  }
}