package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class StorageSystem extends SubsystemBase {
  private boolean storageOn = false;
  private PIDMotor leftMotor;
  private PIDMotor rightMotor;

  public StorageSystem() {
    SlotConfigs configs = new SlotConfigs();
    configs.kP = Constants.StorageRollers.kP;
    configs.kI = Constants.StorageRollers.kI;
    configs.kD = Constants.StorageRollers.kD;

    MotionMagicConfigs mmconfigs = new MotionMagicConfigs();
    mmconfigs.MotionMagicAcceleration = Constants.StorageRollers.MAX_ACCELERATION_RPSPS;
    mmconfigs.MotionMagicCruiseVelocity = Constants.StorageRollers.MAX_VELOCITY_RPS;

    leftMotor = PIDMotor.init(Constants.StorageRollers.MOTOR_ID_LEFT, configs, mmconfigs, 1.0,
        Constants.StorageRollers.LEFT_DIRECTION, 40);

    rightMotor = PIDMotor.init(Constants.StorageRollers.MOTOR_ID_RIGHT, configs, mmconfigs, 1.0,
        Constants.StorageRollers.RIGHT_DIRECTION, 40);

  }

  @Override
  public void periodic() {
    leftMotor.update();
    rightMotor.update();
  }

  public void toggleStorage() {
    this.storageOn = !this.storageOn;
    System.out.println(this.storageOn);

    if (this.storageOn) {
      leftMotor.setVelocity(Constants.StorageRollers.VELOCITY_RPS);
      rightMotor.setVelocity(Constants.StorageRollers.VELOCITY_RPS);
    } else {
      leftMotor.setVelocity(0);
      rightMotor.setVelocity(0);
    }
  }
}
