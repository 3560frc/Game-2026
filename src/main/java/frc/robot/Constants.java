package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {
    public class Climber {
        public static final int MOTOR_ID = 57;
        public static final double kP = 10;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double GEAR_RATIO = 1.0 / 60;
        public static final double MAX_VELOCITY_RPS = 60;
        public static final double MAX_ACCELERATION_RPSPS = 30;
        public static final double ROTATIONS_PER_EXTENSION = 148; // estimate
        public static final InvertedValue DIRECTION = InvertedValue.CounterClockwise_Positive;
    }

    public class IntakeHinge {
        public static final int MOTOR_ID = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double GEAR_RATIO = 1.0 / 12;
        public static final double MAX_VELOCITY_RPS = 0;
        public static final double MAX_ACCELERATION_RPSPS = 0;
        public static final double ROTATIONS_PER_EXTENSION = 0.5; // estimate
        public static final InvertedValue DIRECTION = InvertedValue.Clockwise_Positive;
    }

    public class IntakeRoller {
        public static final int MOTOR_ID = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double VELOCITY_RPS = 10; // estimate
        public static final InvertedValue DIRECTION = InvertedValue.Clockwise_Positive;
    }

    public class ShooterRoller {
        public static final int MOTOR_ID = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double VELOCITY_RPS = 0;
        public static final InvertedValue DIRECTION = InvertedValue.Clockwise_Positive;
    }
}
