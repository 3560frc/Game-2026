package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class CommandClimber extends SubsystemBase {
    PIDMotor motor;

    public CommandClimber() {
        SlotConfigs configs = new SlotConfigs();
        configs.kP = Constants.Climber.kP;
        configs.kI = Constants.Climber.kI;
        configs.kD = Constants.Climber.kD;
        configs.kG = Constants.Climber.kG;
        configs.kS = Constants.Climber.kS;

        MotionMagicConfigs mmconfigs = new MotionMagicConfigs();
        mmconfigs.MotionMagicAcceleration = Constants.Climber.MAX_ACCELERATION_RPSPS;
        mmconfigs.MotionMagicCruiseVelocity = Constants.Climber.MAX_VELOCITY_RPS;

        motor = PIDMotor.init(Constants.Climber.MOTOR_ID, configs, mmconfigs, 1.0, Constants.Climber.DIRECTION);
    }
    
    // IDK how climber actually works rn so this is subject to change
    public void setClimbExtended(boolean extended) {
        if (extended) {
            motor.set(Constants.Climber.ROTATIONS_PER_EXTENSION);
        } else {
            motor.set(0);
        }
    }
}