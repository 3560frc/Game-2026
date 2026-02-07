package frc.robot;

public class Constants {
    public static class Climber {
        public static final int Motor_ID = 57;
        public static final double MOTOR_SPEED = 10;
        public static final double GEARBOX_RATIO = 1.0 / 50.0;
        public static final double kP = 0.1;
        public static final double kI = 0.01;
        public static final double kD = 0.001;
    }

    public static class Intake {
        public static final int Moter_ID = 38;
        public static final int Hinge_Moter_ID = 39;
        public static final double MOTOR_SPEED = 5;
    }

    public static class Shooter {
        public static final int Moter_ID_1 = 0;
        public static final double MOTOR_SPEED_1 = 1;

        public static final int Motor_ID_2 = 1;
        public static final double MOTOR_SPEED_2 = 1;
    }
}