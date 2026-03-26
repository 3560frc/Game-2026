package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ShooterSystem extends SubsystemBase {
  PIDMotor shooterRoller;
  PIDMotor shooterFeed;
  public boolean reverseDirection = false;
  public boolean shooting = false;
  public boolean feeding = false;
  private double targetVelocity = Constants.ShooterRoller.VELOCITY_RPS;

  public ShooterSystem() {
    SlotConfigs rollerconfigs = new SlotConfigs();
    rollerconfigs.kP = Constants.ShooterRoller.kP;
    rollerconfigs.kI = Constants.ShooterRoller.kI;
    rollerconfigs.kD = Constants.ShooterRoller.kD;

    MotionMagicConfigs mmrollerconfigs = new MotionMagicConfigs();
    mmrollerconfigs.MotionMagicAcceleration = Constants.ShooterRoller.MAX_ACCELERATION_RPSPS;
    mmrollerconfigs.MotionMagicCruiseVelocity = Constants.ShooterRoller.MAX_VELOCITY_RPS;

    shooterRoller = PIDMotor.init(Constants.ShooterRoller.MOTOR_ID, rollerconfigs, mmrollerconfigs, 1.0,
        Constants.ShooterRoller.DIRECTION, 100);

    SlotConfigs feedconfigs = new SlotConfigs();
    feedconfigs.kP = Constants.ShooterFeed.kP;
    feedconfigs.kI = Constants.ShooterFeed.kI;
    feedconfigs.kD = Constants.ShooterFeed.kD;

    MotionMagicConfigs mmfeedconfigs = new MotionMagicConfigs();
    mmfeedconfigs.MotionMagicAcceleration = Constants.ShooterFeed.MAX_ACCELERATION_RPSPS;
    mmfeedconfigs.MotionMagicCruiseVelocity = Constants.ShooterFeed.MAX_VELOCITY_RPS;

    shooterFeed = PIDMotor.init(Constants.ShooterFeed.MOTOR_ID, feedconfigs, mmfeedconfigs, 1.0,
        Constants.ShooterFeed.DIRECTION, 100);
  }

  @Override
  public void periodic() {
    shooterRoller.update();
    shooterFeed.update();
  }

  public void setShooterVelocity(double velocity) {
    this.targetVelocity = velocity;
  }

  public void resetShooterVelocity() {
    this.targetVelocity = Constants.ShooterRoller.VELOCITY_RPS;
  }

  public void toggleFeed() {
    this.feeding = !this.feeding;

    if (this.feeding) {
      if (!reverseDirection) {
        shooterFeed.setVelocity(Constants.ShooterFeed.VELOCITY_RPS);
      } else {
        shooterFeed.setVelocity(-Constants.ShooterFeed.VELOCITY_RPS);
      }
    } else {
      shooterFeed.setVelocity(0);
    }
  }

  public void toggleShooter() {
    this.shooting = !this.shooting;

    if (this.shooting) {
      if (!reverseDirection) {
        shooterRoller.setVelocity(this.targetVelocity);
      } else {
        shooterRoller.setVelocity(-this.targetVelocity);
      }
    } else {
      shooterRoller.setVelocity(0);
    }
  }

  public void activateShooterWithSpeed(double percent) {
    this.shooting = percent > 0;
    shooterRoller.setVelocity((reverseDirection ? -1 : 1) * this.targetVelocity * percent);
  }

  public void activateFeedWithSpeed(double percent) {
    this.feeding = percent > 0;
    shooterFeed.setVelocity((reverseDirection ? -1 : 1) * this.targetVelocity * percent);
  }
}