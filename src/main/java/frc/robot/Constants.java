package frc.robot;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int BLINKIN1 = 9;
    public static final int BLINKIN2 = 0;


    public static final class Shooter {
        public static final int SHOOTER_MOTOR_CAN = 100; //FIXME
        public static final int SHOOTER_LEFT_PIVOT_CAN = 100; //FIXME
        public static final int SHOOTER_RIGHT_PIVOT_CAN = 100; //FIXME


        public static final double maxShoulderVelocity = 0.9;
        public static final double maxShoulderAcceleration = 10;
        public static final double maxArmVelocity = .7;
        public static final double maxArmAcceleration = 1;
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
    }

    public static final class Climber{
        public static int CLIMBER_LEFT_CAN = 100; //FIXME
        public static int CLIMBER_RIGHT_CAN = 100; //FIXME
        public static double climberDownPosition = -0.1;
        public static double climberUpPosition = 5;
    }
}
