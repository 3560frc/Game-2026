package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PIDMotor;

public class CommandClimber extends SubsystemBase {
    public static final int Motor_ID = 57;
    public static final double MOTOR_SPEED = 10;
    public static final double GEARBOX_RATIO = 1.0 / 50.0;

    public static final double kP = 0.1;
    public static final double kI = 0.01;
    public static final double kD = 0.001;

    public enum ClimberState {
        UP,
        DOWN,
        STOP
    }

    private static final PIDMotor motor = new PIDMotor().init(Motor_ID, kP, kI, kD, false, null);


    public void setMovement(ClimberState state) {
        switch (state) {
            case UP:
                motor.setVelocity(MOTOR_SPEED * GEARBOX_RATIO);
                break;
            case DOWN:
                motor.setVelocity(-MOTOR_SPEED * GEARBOX_RATIO);
                break;
            case STOP:
                motor.setVelocity(0);
                break;
            default:
                break;
        }
    };
}