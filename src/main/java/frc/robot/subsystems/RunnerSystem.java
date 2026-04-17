package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PIDMotor;

public class RunnerSystem extends SubsystemBase {
  PIDMotor motors[];

  @Override
  public void periodic() {
    for (int i = 0; i < motors.length; i++) {
      motors[i].update();
    }
  }
}
