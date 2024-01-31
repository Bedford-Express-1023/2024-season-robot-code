package frc.robot;

//import com.ctre.phoenix.motorcontrol.NeutralMode;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int BLINKIN1 = 9;
    public static final int BLINKIN2 = 0;

    public static final class Shooter {
        public static final int SHOOTER_MOTOR_CAN = 100; //FIXME
        public static final int SHOOTER_LEFT_PIVOT_CAN = 100; //FIXME
        public static final int SHOOTER_RIGHT_PIVOT_CAN = 100; //FIXME

        public static final double minPivotAngle = 0; //FIXME
        public static final double maxPivotAngle = 0; //FIXME
        public static final double targetShooterPivotIndexAngle = 0; //FIXME
        public static final double minShooterPivotIndexAngle = 0; //FIXME
        public static final double maxShooterPivotIndexAngle = 0; //FIXME

        public static final double shooterVelocityAmplifierConstant = 0; //FIXME
        public static final double shooterVelocityPlatformConstant = 0; //FIXME

    }

    public static final class Indexer {
        public static final int INDEXER_CAN = 100; //FIXME
        public static final int INDEXER_BEAM_BREAK_DIO = 10; //FIXME
    }

    public static final class Intake{
        public static int INTAKE_CAN = 100;
        public static int INTAKE_PIVOT_CAN = 100;

        public static double maxWristVelocity = .5;
        public static double maxWristAcceleration = 15;

        public static double intakeSpeed = 0.8;
        public static double intakeUpPosition = 0;
        public static double intakeDownPosition = 0.3;
        public static double targetIntakePivotIndexAngle = 0;
        public static double minIntakePivotIndexAngle = 0; //FIXME
        public static double maxIntakePivotIndexAngle = 0; //FIXME
    }

    public static final class Climber{
        public static int CLIMBER_LEFT_CAN = 100; //FIXME
        public static int CLIMBER_RIGHT_CAN = 100; //FIXME
        public static double climberDownPosition = -0.1;
        public static double climberUpPosition = 5;
        public static double climberStatorCurrentLimit = 5;
    }
}
