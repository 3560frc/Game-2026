package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class CommandShooter extends SubsystemBase {
  PIDMotor intakeMotor = PIDMotor.initVelocityOnly(
    Constants.ShooterRoller.MOTOR_ID, 
    Constants.ShooterRoller.kP, 
    Constants.ShooterRoller.kI, 
    Constants.ShooterRoller.kD, 
    InvertedValue.Clockwise_Positive
  );

  public void setState(boolean enabled) {
    if (enabled) {
      intakeMotor.setVelocity(5);
    } else {
      intakeMotor.setVelocity(0);
    }
  }
}