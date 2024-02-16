// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Shooter.ShootAtAmplifier;
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

  
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController DriverController = new CommandXboxController(1);
   private final CommandXboxController ManipulatorController = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; //  drivetrain
  Limelight limelightSubsystem = new Limelight();
  IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
   ShooterSubsystem ShooterSubsystem = new ShooterSubsystem();
  IndexerSubsystem IndexerSubsystem = new IndexerSubsystem();
  private final FieldCentric drive = new FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 15% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric, driving in open loop
  ShootAtAmplifier shooterAtAmplifier = new ShootAtAmplifier();
  private final SwerveDriveBrake brake = new SwerveDriveBrake();
  private final PointWheelsAt point = new PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final DigitalInput indexerBeamBreak = new DigitalInput(Constants.Indexer.INDEXER_BEAM_BREAK_DIO); 

    public RobotContainer() { /* 
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-LeftYAxis * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-LeftXAxis * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-RightXAxis * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));*/
      ShooterSubsystem.setDefaultCommand(ShooterSubsystem.StopShooter());
      //IntakeSubsystem.setDefaultCommand(IntakeSubsystem.IntakeStop());
    DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    DriverController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-LeftYAxis, -LeftXAxis))));
    configureBindings();
    DriverController.x().whileTrue(limelightSubsystem.RotateWithLimelight()); 
    
   ManipulatorController.a().whileTrue(IntakeSubsystem.IntakeNote()).whileFalse(IntakeSubsystem.IntakeStop());
   ManipulatorController.b().whileTrue(ShooterSubsystem.ShootAtAmplifier()).whileFalse(ShooterSubsystem.StopShooter());
   ManipulatorController.x().whileTrue(IndexerSubsystem.FeedShooter()).whileFalse(IndexerSubsystem.StopIndex());
  // ManipulatorController.x().whileTrue(IntakeSubsystem.IntakeDown()).whileFalse(IntakeSubsystem.IntakePivotStop());
   ManipulatorController.y().whileTrue(IntakeSubsystem.IntakeDown()).whileFalse(IntakeSubsystem.IntakePrepareToIndex());
   ManipulatorController.leftBumper().whileTrue(ShooterSubsystem.PointTowardsSpeaker());
/* 
      if (ManipulatorController.getBButton()) {
        IntakeSubsystem.IntakeDown();
        IntakeSubsystem.IntakeNote();
      }
      else {
        IntakeSubsystem.IntakePrepareToIndex(); 
        ShooterSubsystem.ShooterPrepareToIndex();
      }
    }
    
      
  if (ManipulatorControllor.getYButton()) {
    if (IntakeSubsystem.intakeReadyToIndex && ShooterSubsystem.shooterReadyToIndex) {
      IntakeSubsystem.IntakeNote();
      IndexerSubsystem.IndexNote();
      if (indexerBeamBreakValue = false) {
        IndexerSubsystem.StopIndex();
        IntakeSubsystem.IntakeStop();
      }
      else {
      IntakeSubsystem.IntakeStop();
      IndexerSubsystem.StopIndex();
      }
    }
  }
    if (ManipulatorController.getAButton()) {
      ShooterSubsystem.ShootAtAmplifier();
      if(shooterCurrentRPMValue > Constants.Shooter.shooterVelocityAmplifierConstant - 100 && shooterCurrentRPMValue < Constants.Shooter.shooterVelocityAmplifierConstant + 100) {
        IndexerSubsystem.IndexNote();
      }
      else {
        IndexerSubsystem.StopIndex();
      }
    }
  }
*/}
  private void configureBindings() {
    // reset the field-centric heading on left bumper press
    DriverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  @Override
  public void periodic() {
    /* 
    LeftXAxis = DriverController.getLeftX();
    LeftYAxis = DriverController.getLeftY();
    RightXAxis = DriverController.getRightX();
    RightYAxis = DriverController.getRightY();
    */
    
    SmartDashboard.putNumber("LeftxAxis", LeftXAxis);
    SmartDashboard.putNumber("LeftYAxis", LeftYAxis);
    SmartDashboard.putNumber("RightxAxis", RightXAxis);
    SmartDashboard.putNumber("RightYaxis", RightYAxis);

    // BButton = ManipulatorController.b().getAsBoolean();
    // AButton = ManipulatorController.a().getAsBoolean();
    // YButton = ManipulatorController.y().getAsBoolean();

    indexerBeamBreakValue = indexerBeamBreak.get();
    shooterCurrentRPMValue = ShooterSubsystem.shooterCurrentRPM;
  }

}
