package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ShooterSystem extends SubsystemBase {
  PIDMotor intakeMotor = PIDMotor.initVelocityOnly(
    Constants.ShooterRoller.MOTOR_ID, 
    Constants.ShooterRoller.kP, 
    Constants.ShooterRoller.kI, 
    Constants.ShooterRoller.kD, 
    InvertedValue.Clockwise_Positive
  );

  public Command setState(boolean enabled) {
    return run(() -> {
      if (enabled) {
        intakeMotor.setVelocity(5);
      } else {
        intakeMotor.setVelocity(0);
      }
    });
  }
}