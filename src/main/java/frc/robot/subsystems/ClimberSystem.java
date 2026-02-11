package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ClimberSystem extends SubsystemBase {
    PIDMotor motor;
    boolean extended;

    @Override
    public void periodic() {
        motor.update();
    }

    public ClimberSystem() {
        SlotConfigs configs = new SlotConfigs();
        configs.kP = Constants.Climber.kP;
        configs.kI = Constants.Climber.kI;
        configs.kD = Constants.Climber.kD;
        configs.kG = Constants.Climber.kG;
        configs.kS = Constants.Climber.kS;
        configs.GravityType = GravityTypeValue.Elevator_Static;

        MotionMagicConfigs mmconfigs = new MotionMagicConfigs();
        mmconfigs.MotionMagicAcceleration = Constants.Climber.MAX_ACCELERATION_RPSPS;
        mmconfigs.MotionMagicCruiseVelocity = Constants.Climber.MAX_VELOCITY_RPS;

        motor = PIDMotor.init(Constants.Climber.MOTOR_ID, configs, mmconfigs, 1.0, Constants.Climber.DIRECTION);
    }

    public Command setClimbExtended(boolean extended) {
        return run(() -> {
            if (extended) {
                motor.set(Constants.Climber.ROTATIONS_PER_EXTENSION);
            } else {
                motor.set(0);
            }
        });
    }

    public Command toggleClimbExtended() {
        return run(() -> {
            extended = !extended;

            if (extended) {
                motor.set(Constants.Climber.ROTATIONS_PER_EXTENSION);
            } else {
                motor.set(0);
            }
        });
    }
}