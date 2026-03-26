package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class IntakeSystem extends SubsystemBase {
  PIDMotor leftHingeMotor;
  PIDMotor rightHingeMotor;
  PIDMotor intakeMotor;
  // SG: nit -> use enum?
  int intakeState = 0; // -1 = down, 0 = stop, 1 = up
  boolean intakeRollersEnabled;

  public IntakeSystem() {
    SlotConfigs configs = new SlotConfigs();
    configs.kP = Constants.IntakeRoller.kP;
    configs.kI = Constants.IntakeRoller.kI;
    configs.kD = Constants.IntakeRoller.kD;

    MotionMagicConfigs mmconfigs = new MotionMagicConfigs();
    mmconfigs.MotionMagicAcceleration = Constants.IntakeRoller.MAX_ACCELERATION_RPSPS;
    mmconfigs.MotionMagicCruiseVelocity = Constants.IntakeRoller.MAX_VELOCITY_RPS;

    intakeMotor = PIDMotor.init(Constants.IntakeRoller.MOTOR_ID, configs, mmconfigs, 1.0,
        Constants.IntakeRoller.DIRECTION, 100);

    SlotConfigs left_configs = new SlotConfigs();
    left_configs.kP = Constants.IntakeHinge.kP;
    left_configs.kI = Constants.IntakeHinge.kI;
    left_configs.kD = Constants.IntakeHinge.kD;
    left_configs.kG = Constants.IntakeHinge.kG;
    left_configs.kS = Constants.IntakeHinge.kS;

    MotionMagicConfigs mmleft_configs = new MotionMagicConfigs();
    mmleft_configs.MotionMagicAcceleration = Constants.IntakeHinge.MAX_ACCELERATION_RPSPS;
    mmleft_configs.MotionMagicCruiseVelocity = Constants.IntakeHinge.MAX_VELOCITY_RPS;

    leftHingeMotor = PIDMotor.init(Constants.IntakeHinge.MOTOR_ID_LEFT, left_configs, mmleft_configs, 1.0,
        Constants.IntakeHinge.LEFT_DIRECTION);

    SlotConfigs right_configs = new SlotConfigs();
    right_configs.kP = Constants.IntakeHinge.kP;
    right_configs.kI = Constants.IntakeHinge.kI;
    right_configs.kD = Constants.IntakeHinge.kD;
    right_configs.kG = Constants.IntakeHinge.kG;
    right_configs.kS = Constants.IntakeHinge.kS;

    MotionMagicConfigs mmright_configs = new MotionMagicConfigs();
    mmright_configs.MotionMagicAcceleration = Constants.IntakeHinge.MAX_ACCELERATION_RPSPS;
    mmright_configs.MotionMagicCruiseVelocity = Constants.IntakeHinge.MAX_VELOCITY_RPS;

    rightHingeMotor = PIDMotor.init(Constants.IntakeHinge.MOTOR_ID_RIGHT, right_configs, mmright_configs, 1.0,
        Constants.IntakeHinge.RIGHT_DIRECTION);
  }

  @Override
  public void periodic() {
    intakeMotor.update();
    leftHingeMotor.update();
    rightHingeMotor.update();
  }

  public void toggleIntakeRollers() {
    intakeRollersEnabled = !intakeRollersEnabled;

    if (intakeRollersEnabled) {
      intakeMotor.setVelocity(Constants.IntakeRoller.VELOCITY_RPS);
    } else {
      intakeMotor.setVelocity(0);
    }
  }

  public void setIntakeHingeDown() {
    intakeState = -1;
    leftHingeMotor.set(Constants.IntakeHinge.ROTATIONS_PER_EXTENSION);
    rightHingeMotor.set(Constants.IntakeHinge.ROTATIONS_PER_EXTENSION);
  }

  public void setIntakeHingeUp() {
    intakeState = 1;
    leftHingeMotor.set(-Constants.IntakeHinge.ROTATIONS_PER_EXTENSION);
    rightHingeMotor.set(-Constants.IntakeHinge.ROTATIONS_PER_EXTENSION);
  }

  public void stopIntakeHinge() {
    intakeState = 0;
    leftHingeMotor.stop();
    rightHingeMotor.stop();
  }

  // public void toggleIntakeExtended() {
  // intakeUp = !intakeUp;
  // System.out.println(intakeUp);
  // if (intakeUp) {
  // leftHingeMotor.set(Constants.IntakeHinge.ROTATIONS_PER_EXTENSION);
  // rightHingeMotor.set(Constants.IntakeHinge.ROTATIONS_PER_EXTENSION);
  // } else {
  // rightHingeMotor.stop();
  // leftHingeMotor.stop();
  // }
  // }
}
