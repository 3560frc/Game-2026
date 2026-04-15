package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import frc.robot.Constants;
import frc.robot.PIDMotor;

public class StorageSystem {
  private boolean storageOn = false;
  private PIDMotor leftMotor;
  private PIDMotor rightMotor;

  public StorageSystem() {
    SlotConfigs left_configs = new SlotConfigs();
    left_configs.kP = Constants.StorageRollers.kP;
    left_configs.kI = Constants.StorageRollers.kI;
    left_configs.kD = Constants.StorageRollers.kD;

    MotionMagicConfigs mmleft_configs = new MotionMagicConfigs();
    mmleft_configs.MotionMagicAcceleration = Constants.StorageRollers.MAX_ACCELERATION_RPSPS;
    mmleft_configs.MotionMagicCruiseVelocity = Constants.StorageRollers.MAX_VELOCITY_RPS;

    leftMotor = PIDMotor.init(Constants.StorageRollers.MOTOR_ID_LEFT, left_configs, mmleft_configs, 1.0,
        Constants.StorageRollers.DIRECTION);

    SlotConfigs right_configs = new SlotConfigs();
    right_configs.kP = Constants.StorageRollers.kP;
    right_configs.kI = Constants.StorageRollers.kI;
    right_configs.kD = Constants.StorageRollers.kD;

    MotionMagicConfigs mmright_configs = new MotionMagicConfigs();
    mmright_configs.MotionMagicAcceleration = Constants.StorageRollers.MAX_ACCELERATION_RPSPS;
    mmright_configs.MotionMagicCruiseVelocity = Constants.StorageRollers.MAX_VELOCITY_RPS;

    rightMotor = PIDMotor.init(Constants.StorageRollers.MOTOR_ID_RIGHT, right_configs, mmright_configs, 1.0,
        Constants.StorageRollers.DIRECTION);
  }

  public void toggleStorage() {
    this.storageOn = !this.storageOn;
    System.out.println(this.storageOn);

    if (this.storageOn) {
      leftMotor.setVelocity(Constants.StorageRollers.VELOCITY_RPS);
      rightMotor.setVelocity(Constants.StorageRollers.VELOCITY_RPS);
    } else {
      leftMotor.stop();
      rightMotor.stop();
    }
  }
}
