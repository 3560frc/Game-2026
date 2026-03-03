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

  IntakeState state;

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
    configs.kP = Constants.IntakeRoller.kP;
    configs.kI = Constants.IntakeRoller.kI;
    configs.kD = Constants.IntakeRoller.kD;

    MotionMagicConfigs mmconfigs1 = new MotionMagicConfigs();
    mmconfigs.MotionMagicAcceleration = Constants.IntakeRoller.MAX_ACCELERATION_RPSPS;
    mmconfigs.MotionMagicCruiseVelocity = Constants.IntakeRoller.MAX_VELOCITY_RPS;

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

  public enum IntakeState {
    HOVERING,
    RETRACTED,
    DOWN
  }

  public Command toggleIntakeExtended() {
    if (this.state == IntakeState.HOVERING) {
      this.state = IntakeState.RETRACTED;
    } else {
      this.state = IntakeState.HOVERING;
    }

    return setIntakeExtended(this.state);
  }

  public Command setIntakeExtended(IntakeState state) {
    return run(() -> {
      switch (state) {
        case HOVERING:
          hingeMotor.set(Constants.IntakeHinge.HOVER_SETPOINT_ROTATIONS);
          break;
        case RETRACTED:
          hingeMotor.set(Constants.IntakeHinge.RETRACTED_SETPOINT_ROTATIONS);
          break;
        case DOWN:
          hingeMotor.set(0);
          break;
      }
    });
  }
}
