package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class IntakeSystem extends SubsystemBase {
  PIDMotor hingeMotor;
  PIDMotor intakeMotor;
  boolean intakeUp;

  public IntakeSystem() {
    SlotConfigs configs = new SlotConfigs();
    configs.kP = Constants.IntakeHinge.kP;
    configs.kI = Constants.IntakeHinge.kI;
    configs.kD = Constants.IntakeHinge.kD;
    configs.kG = Constants.IntakeHinge.kG;
    configs.kS = Constants.IntakeHinge.kS;

    MotionMagicConfigs mmconfigs = new MotionMagicConfigs();
    mmconfigs.MotionMagicAcceleration = Constants.IntakeHinge.MAX_ACCELERATION_RPSPS;
    mmconfigs.MotionMagicCruiseVelocity = Constants.IntakeHinge.MAX_VELOCITY_RPS;

    hingeMotor = PIDMotor.init(Constants.IntakeHinge.MOTOR_ID, configs, mmconfigs, 1.0,
        Constants.IntakeHinge.DIRECTION);

    SlotConfigs configs1 = new SlotConfigs();
    configs1.kP = Constants.IntakeRoller.kP;
    configs1.kI = Constants.IntakeRoller.kI;
    configs1.kD = Constants.IntakeRoller.kD;

    MotionMagicConfigs mmconfigs1 = new MotionMagicConfigs();
    mmconfigs1.MotionMagicAcceleration = Constants.IntakeRoller.MAX_ACCELERATION_RPSPS;
    mmconfigs1.MotionMagicCruiseVelocity = Constants.IntakeRoller.MAX_VELOCITY_RPS;

    intakeMotor = PIDMotor.init(Constants.IntakeRoller.MOTOR_ID, configs1, mmconfigs1, 1.0,
        Constants.IntakeRoller.DIRECTION);
  }

  public enum IntakeDirection {
    FORWARD,
    REVERSE,
    STOP
  }

  @Override
  public void periodic() {
    intakeMotor.update();
    hingeMotor.update();
  }

  public Command setIntakeRollerEnabled(boolean enabled, IntakeDirection direction) {
    return run(() -> {
      System.out.println(direction);
      System.out.println(enabled);

      if (enabled) {
        if (direction == IntakeDirection.FORWARD) {
          intakeMotor.setVelocity(Constants.IntakeRoller.VELOCITY_RPS);
        }

        if (direction == IntakeDirection.REVERSE) {
          intakeMotor.setVelocity(-Constants.IntakeRoller.VELOCITY_RPS);
        }
      } else {
        intakeMotor.setVelocity(0);
      }
    });
  }

  public void toggleIntakeExtended() {
    intakeUp = !intakeUp;
    System.out.println(intakeUp);
    if (intakeUp) {
      hingeMotor.set(Constants.IntakeHinge.ROTATIONS_PER_EXTENSION);
    } else {
      hingeMotor.set(0);
    }
  }
}
