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
        public static final double targetShooterPivotIndexAngle = 0.255; 
        public static final double minShooterPivotIndexAngle = 0; //FIXME
        public static final double maxShooterPivotIndexAngle = 0; //FIXME
        public static final double shooterVelocitySubwooferConstant = 4000; //tested and works
        public static final double shooterAngleSubwooferConstant = 0.029; //tested and works
        public static final double shooterVelocityPlatformConstant = 0.203;
        public static final double shooterAngleFarshotConstant = 0.251;
        public static final double[][] pivotTable = {{0.029, 0.017},
                                                    {.033,.0164},// x is the shooter angle  y in the distance
                                                    {.066, .0144},    //SUBTRACT 0.028 FROM ALL
                                                    {.0783, .0129},
                                                    {.1, .0117},
                                                    {.127, .0106},
                                                    {.156,.009},
                                                    {.167,.008},
                                                    {.178,.0074},
                                                    {.1877,.0069},
                                                    {.203,.0062},
                                                    {.214, .0057},
                                                    {.219,.0055},
                                                    {.226,.0052},
                                                    {.235,.0047},
                                                    {.246,.0042},
                                                    {.251,.004},
                                                    {.253,.0038},
                                                    {.257,.0037}
                                                        }; //x value is distance with limelight, y is shooter angle
        public static final double targetAmpAngle = 0.029; //FIXME
        public static final double ampRPM = 2700;
    }

    public static final class Indexer {
        public static final int INDEXER_CAN = 2; //FIXME
        public static final int INDEXER_BEAM_BREAK_DIO = 3; //FIXME
    }

    public static final class Intake{
        public static int INTAKE_CAN = 0;
        public static int INTAKE_PIVOT_CAN = 5;
        public static int INTAKE_ENCODER_CAN = 1; //FIXME

        public static double maxWristVelocity = .5;
        public static double maxWristAcceleration = 15;

        public static double intakeSpeed = 0.8;
        public static double intakeUpPosition = -0.025;
        public static double intakeDownPosition = 0.0;
        public static double targetIntakePivotIndexAngle = 0.29;
        public static double minIntakePivotIndexAngle = 0; //FIXME
        public static double maxIntakePivotIndexAngle = 0; //FIXME
    }

    public static final class Climber{
        public static int CLIMBER_LEFT_CAN = 7; //FIXME
        public static int CLIMBER_RIGHT_CAN = 6; //FIXME
        public static double climberDownPosition = -0.1;
        public static double climberUpPosition = 5;
        public static double climberStatorCurrentLimit = 5;
    }
}
