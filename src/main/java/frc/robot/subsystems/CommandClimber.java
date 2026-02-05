package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CommandClimber extends SubsystemBase {
    private final TalonFX climberMoter = new TalonFX(Constants.Climber.Moter_ID);

    public void setSpeed(double speed) {
        climberMoter.set(speed * Constants.Climber.GEARBOX_RATIO);
    }
}