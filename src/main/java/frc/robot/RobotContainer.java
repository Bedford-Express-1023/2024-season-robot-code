// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Indexer.IndexNote;
import frc.robot.Commands.Shooter.ShootAtAmplifier;
import frc.robot.Constants.Intake;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer extends SubsystemBase {
  public static double LeftXAxis;
  public static double LeftYAxis;
  public static double RightXAxis;
  public static double RightYAxis;
  public static double shooterCurrentRPMValue;
  public static boolean BButton;
  public static boolean AButton;
  public static boolean YButton;

  public static boolean indexerBeamBreakValue;

CANcoder leftFrontCANcoder = new CANcoder(4, "1023");
CANcoder rightFrontCANcoder = new CANcoder(5, "1023");
CANcoder leftBackCANcoder = new CANcoder(3 ,"1023");
CANcoder rightBackCANcoder = new CANcoder(2, "1023");
  
  private double MaxSpeed = 3; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandXboxController DriverController = new CommandXboxController(0);
  XboxController Conttroller = new XboxController(0);
  // private final XboxController ManipulatorController = new XboxController(0); 
      private final CommandXboxController ManipulatorController = new CommandXboxController(1);// My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; //  drivetrain
  Limelight limelightSubsystem = new Limelight();
  IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
   ShooterSubsystem ShooterSubsystem = new ShooterSubsystem();
  IndexerSubsystem IndexerSubsystem = new IndexerSubsystem();
  private final FieldCentric drive = new FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 15% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric, driving in open loop
  ShootAtAmplifier shooterAtAmplifier = new ShootAtAmplifier();
  private final SwerveDriveBrake brake = new SwerveDriveBrake();
  private final PointWheelsAt point = new PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
double check = 5;
  //private final DigitalInput indexerBeamBreak = new DigitalInput(0); 

    public RobotContainer() { 
ShooterSubsystem.setDefaultCommand(ShooterSubsystem.ShooterPrepareToIndex());
IntakeSubsystem.setDefaultCommand(IntakeSubsystem.IntakePrepareToIndex());
 
    DriverController.x().whileTrue(limelightSubsystem.RotateWithLimelight()); 
   ManipulatorController.a().whileTrue(IntakeSubsystem.IntakeNote()).whileFalse(IntakeSubsystem.IntakeStop());
   ManipulatorController.b().whileTrue(ShooterSubsystem.ShooterShoot()).whileFalse(ShooterSubsystem.StopShooter());
   ManipulatorController.leftBumper().whileTrue(IndexerSubsystem.FeedShooter()).whileFalse(IndexerSubsystem.StopIndex());
   ManipulatorController.rightBumper().whileTrue(IndexerSubsystem.ReverseIndexer()).whileFalse(IndexerSubsystem.StopIndex());
   ManipulatorController.pov(180).whileTrue(IntakeSubsystem.IntakeDown()).whileFalse(IntakeSubsystem.IntakePrepareToIndex());
      ManipulatorController.pov(0).whileTrue(IntakeSubsystem.IntakeRun()).whileFalse(IntakeSubsystem.IntakePrepareToIndex());
   ManipulatorController.pov(90).whileTrue(ShooterSubsystem.ShootAtSubwoofer()).whileFalse(ShooterSubsystem.ShooterPrepareToIndex());
   ManipulatorController.y().whileTrue(ShooterSubsystem.ShootInAmp()).whileFalse(ShooterSubsystem.StopShooter());
   //ManipulatorController.leftBumper().whileTrue(ShooterSubsystem.PointTowardsSpeaker()).whileFalse(ShooterSubsystem.ShooterPrepareToIndex());
     configureBindings();  
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() -> drive.withVelocityX(-LeftYAxis * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-LeftXAxis * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-RightXAxis*MaxAngularRate)//RightXAxis * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
DriverController.b().whileTrue(drivetrain
    .applyRequest(() -> point.withModuleDirection(new Rotation2d(-DriverController.getLeftY(), -DriverController.getLeftX()))));

// reset the field-centric heading on left bumper press
DriverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

if (Utils.isSimulation()) {
  drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
}
drivetrain.registerTelemetry(logger::telemeterize);
}

   

  @Override
  public void periodic() {
  //   if (ManipulatorController.getBButton() == true && ManipulatorController.getAButton() == false && ManipulatorController.getYButton() == false) {
      
  //       IntakeSubsystem.IntakeNote();
  //      check = 1;
  //     }
  // else if (ManipulatorController.getYButton() == true  && ManipulatorController.getAButton() == false && ManipulatorController.getBButton() == false) {
  //   if (IntakeSubsystem.intakeReadyToIndex && ShooterSubsystem.shooterReadyToIndex) {
  //     IntakeSubsystem.IntakeNote();
  //     IndexerSubsystem.IndexNote();
  //   }
  //     if (indexerBeamBreakValue == false) {
  //       IndexerSubsystem.StopIndex();
  //       IntakeSubsystem.IntakeStop();
  //     }
  //     check = 2;
  // }
  // else if (ManipulatorController.getAButton() == true  && ManipulatorController.getBButton() == false && ManipulatorController.getYButton() == false) {
  //     ShooterSubsystem.ShootAtAmplifier();
  //     check = 3;
  //     if(shooterCurrentRPMValue > Constants.Shooter.shooterVelocityAmplifierConstant - 100 && shooterCurrentRPMValue < Constants.Shooter.shooterVelocityAmplifierConstant + 100) {
  //       IndexerSubsystem.IndexNote();
  //     }
  //     else {
  //       IndexerSubsystem.StopIndex();
  //     }
  //   }
  // else{
  // IndexerSubsystem.StopIndex();
  // ShooterSubsystem.StopShooter();
  // ShooterSubsystem.ShooterPrepareToIndex();
  // IntakeSubsystem.IntakePrepareToIndex();
  // IntakeSubsystem.IntakeStop();
  // check = 0;
  // }

  if ((DriverController.getRightX()>.15 ) || (DriverController.getRightX() <-.15)){
    RightXAxis = DriverController.getRightX();
    }
    else if (Conttroller.getYButton())
        {
         RightXAxis = -limelightSubsystem.rotationtmp;
        }
    else{
      RightXAxis=0;
    } 
    if ((DriverController.getLeftY()>.15 ) || (DriverController.getLeftY() <-.15)){
      LeftYAxis = DriverController.getLeftY();
      }
        else{
      LeftYAxis = 0;
        }
      if ((DriverController.getLeftX()>.15 ) || (DriverController.getLeftX() <-.15)){
        LeftXAxis = DriverController.getLeftX();
        }
          else{
      LeftXAxis=0;
    }
  
    // LeftXAxis = DriverController.getLeftX();
    // LeftYAxis = DriverController.getLeftY();
    // RightXAxis = DriverController.getRightX();
    // RightYAxis = DriverController.getRightY();
    
      SmartDashboard.putNumber("check",check);
    SmartDashboard.putNumber("LeftxAxis", LeftXAxis);
    SmartDashboard.putNumber("LeftYAxis", LeftYAxis);
    SmartDashboard.putNumber("RightxAxis", RightXAxis);
    SmartDashboard.putNumber("RightYaxis", RightYAxis);

    // BButton = ManipulatorController.b().getAsBoolean();
    // AButton = ManipulatorController.a().getAsBoolean();
    // YButton = ManipulatorController.y().getAsBoolean();

    //indexerBeamBreakValue = indexerBeamBreak.get();
    shooterCurrentRPMValue = ShooterSubsystem.shooterCurrentRPM;
   // SmartDashboard.putBoolean("beam break", indexerBeamBreakValue);
    SmartDashboard.putNumber("left front CANcoder", leftFrontCANcoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("right front CANcoder", rightFrontCANcoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("left back CANcoder", leftBackCANcoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("right back CANcoder", rightBackCANcoder.getAbsolutePosition().getValueAsDouble());

  }

}
