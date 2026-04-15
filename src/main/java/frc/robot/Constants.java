package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {
    // public class Climber {
    // public static final int MOTOR_ID = 57;
    // public static final double kP = 12;
    // public static final double kI = 0;
    // public static final double kD = 0;
    // public static final double kG = 0;
    // public static final double kS = 0;
    // public static final double GEAR_RATIO = 1.0;
    // public static final double MAX_VELOCITY_RPS = 20;
    // public static final double MAX_ACCELERATION_RPSPS = 10;
    // public static final double ROTATIONS_PER_EXTENSION = 50; // estimate
    // public static final InvertedValue DIRECTION =
    // InvertedValue.CounterClockwise_Positive;
    // }

    public class IntakeHinge {
        // needs reconfiguration for IDs at least
        public static final int MOTOR_ID_LEFT = 44;
        public static final int MOTOR_ID_RIGHT = 57;
        public static final double kP = 40;
        public static final double kI = 0;
        public static final double kD = 0; // Arm offset 0.06
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double GEAR_RATIO = 1.0;
        public static final double MAX_VELOCITY_RPS = 40;
        public static final double MAX_ACCELERATION_RPSPS = 7;
        public static final double ROTATIONS_PER_EXTENSION = 2.5;
        public static final InvertedValue LEFT_DIRECTION = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue RIGHT_DIRECTION = InvertedValue.Clockwise_Positive;
    }

    public class IntakeRoller {
        public static final int MOTOR_ID = 38;
        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double VELOCITY_RPS = 90; // estimate
        public static final double MAX_VELOCITY_RPS = 120;
        public static final double MAX_ACCELERATION_RPSPS = 610;
        public static final InvertedValue DIRECTION = InvertedValue.CounterClockwise_Positive;
    }

    public class ShooterFeed {
        public static final int MOTOR_ID = 8;
        public static final double kP = 0.029;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double VELOCITY_RPS = 20;
        public static final double MAX_VELOCITY_RPS = 40;
        public static final double MAX_ACCELERATION_RPSPS = 120;
        public static final InvertedValue DIRECTION = InvertedValue.CounterClockwise_Positive;
    }

    public class ShooterRoller {
        public static final int MOTOR_ID = 9;
        public static final double kP = 0.055;
        public static final double kI = 0;
        public static final double kD = 0;
        // public static final double VELOCITY_RPS = 10;
        // public static final double MAX_VELOCITY_RPS = 10;
        public static final double VELOCITY_RPS = 57;
        public static final double MAX_VELOCITY_RPS = 57;
        public static final double MAX_ACCELERATION_RPSPS = 100;
        // The VELOCITY_RPS was 75 WE ARE NOW CHANGING IT FOR TESTING TO 40, THANKS -
        // AIDEN M, YUVRAJ G & AAYUSH K
        public static final InvertedValue DIRECTION = InvertedValue.Clockwise_Positive;
    }

    public class StorageRollers {
        // Needs reconfiguration
        public static final int MOTOR_ID_LEFT = 5;
        public static final int MOTOR_ID_RIGHT = 15;
        public static final double kP = 0.032;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double VELOCITY_RPS = 15;
        public static final double MAX_VELOCITY_RPS = 15;
        public static final double MAX_ACCELERATION_RPSPS = 15;
        public static final InvertedValue DIRECTION = InvertedValue.CounterClockwise_Positive;
    }
}
