package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {
    public class Climber {
        public static final int MOTOR_ID = 57;
        public static final double kP = 12;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double GEAR_RATIO = 1.0 / 60;
        public static final double MAX_VELOCITY_RPS = 60;
        public static final double MAX_ACCELERATION_RPSPS = 30;
        public static final double ROTATIONS_PER_EXTENSION = 200; // estimate
        public static final InvertedValue DIRECTION = InvertedValue.CounterClockwise_Positive;
    }

    public class IntakeHinge {
        public static final int MOTOR_ID = 44;
        public static final double kP = 50;
        public static final double kI = 0;
        public static final double kD = 0;// Arm offset 0.06
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double GEAR_RATIO = 1.0;
        public static final double MAX_VELOCITY_RPS = 20;
        public static final double MAX_ACCELERATION_RPSPS = 7;
        public static final double HOVER_SETPOINT_ROTATIONS = 0.05;
        public static final double RETRACTED_SETPOINT_ROTATIONS = 0.15;
        public static final InvertedValue DIRECTION = InvertedValue.Clockwise_Positive;
    }

    public class IntakeRoller {
        public static final int MOTOR_ID = 38;
        public static final double kP = 0.015;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double VELOCITY_RPS = 60; // estimate
        public static final double MAX_VELOCITY_RPS = 20;
        public static final double MAX_ACCELERATION_RPSPS = 10;
        public static final InvertedValue DIRECTION = InvertedValue.CounterClockwise_Positive;
    }

    public class ShooterTop {
        public static final int MOTOR_ID = 60;
        public static final double kP = 0.032;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double VELOCITY_RPS = 30;
        public static final double MAX_VELOCITY_RPS = 20;
        public static final double MAX_ACCELERATION_RPSPS = 10;
        public static final InvertedValue DIRECTION = InvertedValue.Clockwise_Positive;
    }

    public class ShooterRoller {
        public static final int MOTOR_ID = 9;
        public static final double kP = 0.079;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double VELOCITY_RPS = 65;
        public static final double MAX_VELOCITY_RPS = 20;
        public static final double MAX_ACCELERATION_RPSPS = 10;
        // The VELOCITY_RPS was 75 WE ARE NOW CHANGING IT FOR TESTING TO 40, THANKS -
        // AIDEN M, YUVRAJ G & AAYUSH K
        public static final InvertedValue DIRECTION = InvertedValue.Clockwise_Positive;
    }

    public class StorageRoller {
        public static final int MOTOR_ID = 15;
        public static final double kP = 0.032;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double VELOCITY_RPS = 65;
        public static final double MAX_VELOCITY_RPS = 20;
        public static final double MAX_ACCELERATION_RPSPS = 10;
        public static final InvertedValue DIRECTION = InvertedValue.CounterClockwise_Positive;
    }
}
