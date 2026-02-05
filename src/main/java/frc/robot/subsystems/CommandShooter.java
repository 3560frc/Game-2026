package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class CommandShooter {
  TalonFX m_motor = new TalonFX(5);

  public void runIntake() {
    m_motor.set(0.8);
  }

  public void stopIntake() {
    m_motor.set(0.0);
  }
}
