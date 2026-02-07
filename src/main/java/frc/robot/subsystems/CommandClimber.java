package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CommandClimber extends SubsystemBase {
    private final TalonFX climberMoter = new TalonFX(Constants.Climber.Moter_ID);
    private final PIDController pidController = new PIDController(Constants.Climber.kP, Constants.Climber.kI,
            Constants.Climber.kD);

    public void setSpeed(double speed) {
        climberMotor.set(speed * Constants.Climber.GEARBOX_RATIO);
    }
}