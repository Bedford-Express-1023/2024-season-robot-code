package frc.robot;

//import com.ctre.phoenix.motorcontrol.NeutralMode;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int BLINKIN1 = 9;
    public static final int BLINKIN2 = 0;

    public static final class Shooter {
        public static final int SHOOTER_MOTOR_CAN = 4;
        public static final int SHOOTER_LEFT_PIVOT_CAN = 1; 
        public static final int SHOOTER_RIGHT_PIVOT_CAN = 3;
        public static final int SHOOTER_CANCODER_ID = 0; 

        public static final double minPivotAngle = 0; //FIXME
        public static final double maxPivotAngle = 0; //FIXME
        public static final double targetShooterPivotIndexAngle = -0.08; 
        public static final double minShooterPivotIndexAngle = 0; //FIXME
        public static final double maxShooterPivotIndexAngle = 0; //FIXME
        public static final double shooterVelocitySubwooferConstant = 4000; //tested and works
        public static final double shooterAngleSubwooferConstant = 0.106; //tested and works
        public static final double shooterVelocityPlatformConstant = 0;
        public static final double[][] shooterTable = {{1.4, 4000/60}, //x value is limelight ty, y is shooter RPM
                                                        {1.53, 4000/60},
                                                        {1.61, 4000/60},
                                                        {1.78, 4000/60},
                                                        {2.06, 4000/60},
                                                        {2.1, 4000/60}}; 
        public static final double[][] pivotTable = {{1.4, 0.106},
                                                     {1.53, 0.03},
                                                     {1.61, -0.012},
                                                     {1.78, -0.032},
                                                     {2.06, -0.048},
                                                     {2.1, -0.055}}; //x value is limelight ty, y is shooter angle
        public static final double targetAmpAngle = 0; //FIXME
        public static final double ampRPM = 2700;
    }

    public static final class Indexer {
        public static final int INDEXER_CAN = 2; //FIXME
        public static final int INDEXER_BEAM_BREAK_DIO = 10; //FIXME
    }

    public static final class Intake{
        public static int INTAKE_CAN = 0;
        public static int INTAKE_PIVOT_CAN = 5;
        public static int INTAKE_ENCODER_CAN = 1; //FIXME

        public static double maxWristVelocity = .5;
        public static double maxWristAcceleration = 15;

        public static double intakeSpeed = 0.8;
        public static double intakeUpPosition = -0.026;
        public static double intakeDownPosition = 0.0;
        public static double targetIntakePivotIndexAngle = 0.25;
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
