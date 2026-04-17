package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class IntakeSystem extends SubsystemBase {
  PIDMotor leftHingeMotor;
  PIDMotor rightHingeMotor;
  PIDMotor leftRollerMotor;
  PIDMotor rightRollerMotor;
  // SG: nit -> use enum?
  int intakeState = 0; // -1 = down, 0 = stop, 1 = up
  boolean intakeRollersEnabled;

  public IntakeSystem() {
    SlotConfigs left_configs_roller = new SlotConfigs();
    left_configs_roller.kP = Constants.IntakeRoller.LEFT_kP;
    left_configs_roller.kI = Constants.IntakeRoller.LEFT_kI;
    left_configs_roller.kD = Constants.IntakeRoller.LEFT_kD;

    SlotConfigs right_configs_roller = new SlotConfigs();
    right_configs_roller.kP = Constants.IntakeRoller.RIGHT_kP;
    right_configs_roller.kI = Constants.IntakeRoller.RIGHT_kI;
    right_configs_roller.kD = Constants.IntakeRoller.RIGHT_kD;

    MotionMagicConfigs mmconfigs = new MotionMagicConfigs();
    mmconfigs.MotionMagicAcceleration = Constants.IntakeRoller.MAX_ACCELERATION_RPSPS;
    mmconfigs.MotionMagicCruiseVelocity = Constants.IntakeRoller.MAX_VELOCITY_RPS;

    leftRollerMotor = PIDMotor.init(Constants.IntakeRoller.LEFT_MOTOR_ID, left_configs_roller, mmconfigs, 1.0,
        Constants.IntakeRoller.DIRECTION, 60);

    rightRollerMotor = PIDMotor.init(Constants.IntakeRoller.RIGHT_MOTOR_ID, right_configs_roller, mmconfigs, 1.0,
        Constants.IntakeRoller.DIRECTION, 60);

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
    leftRollerMotor.update();
    rightRollerMotor.update();
    leftHingeMotor.update();
    rightHingeMotor.update();
  }

  public void toggleIntakeRollers() {
    intakeRollersEnabled = !intakeRollersEnabled;

    if (intakeRollersEnabled) {
      leftRollerMotor.setVelocity(Constants.IntakeRoller.VELOCITY_RPS);
      rightRollerMotor.setVelocity(Constants.IntakeRoller.VELOCITY_RPS);
    } else {
      leftRollerMotor.setVelocity(0);
      rightRollerMotor.setVelocity(0);
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
}
