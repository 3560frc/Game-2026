package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class IntakeSystem extends SubsystemBase {
  PIDMotor hingeMotor;
  PIDMotor intakeMotor = PIDMotor.initVelocityOnly(
      Constants.IntakeRoller.MOTOR_ID,
      Constants.IntakeRoller.kP,
      Constants.IntakeRoller.kI,
      Constants.IntakeRoller.kD,
      Constants.IntakeRoller.DIRECTION);

  public IntakeSystem() {
    SlotConfigs configs = new SlotConfigs();
    configs.kP = Constants.IntakeHinge.kP;
    configs.kI = Constants.IntakeHinge.kI;
    configs.kD = Constants.IntakeHinge.kD;
    configs.kG = Constants.IntakeHinge.kG;
    configs.kS = Constants.IntakeHinge.kS;
    configs.GravityType = GravityTypeValue.Arm_Cosine;

    MotionMagicConfigs mmconfigs = new MotionMagicConfigs();
    mmconfigs.MotionMagicAcceleration = Constants.IntakeHinge.MAX_ACCELERATION_RPSPS;
    mmconfigs.MotionMagicCruiseVelocity = Constants.IntakeHinge.MAX_VELOCITY_RPS;

    hingeMotor = PIDMotor.init(Constants.IntakeHinge.MOTOR_ID, configs, mmconfigs, 1.0,
        Constants.IntakeHinge.DIRECTION);
  }

  @Override
  public void periodic() {
    intakeMotor.update();
    hingeMotor.update();
  }

  public Command setIntakeRollerEnabled(boolean enabled) {
    return run(() -> {
      if (enabled) {
        intakeMotor.setVelocity(Constants.IntakeRoller.VELOCITY_RPS);
      } else {
        intakeMotor.setVelocity(0);
      }
    });
  }

  public Command setIntakeExtended(boolean extended) {
    return run(() -> {
      if (extended) {
        hingeMotor.set(Constants.IntakeHinge.ROTATIONS_PER_EXTENSION);
      } else {
        hingeMotor.set(0);
      }
    });
  }
}
