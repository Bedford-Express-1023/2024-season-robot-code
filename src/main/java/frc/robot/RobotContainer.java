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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ClimberMaintainDown;
import frc.robot.Commands.NotePassOff;
import frc.robot.Commands.Climber.ClimberDown;
import frc.robot.Commands.Climber.ClimberUp;
import frc.robot.Commands.Indexer.FeedShooter;
import frc.robot.Commands.Indexer.FeedShooterFast;
import frc.robot.Commands.Indexer.IndexNote;
import frc.robot.Commands.Indexer.ReverseIndexer;
import frc.robot.Commands.Indexer.StopIndex;
import frc.robot.Commands.Intake.IntakeDown;
import frc.robot.Commands.Intake.IntakeNote;
import frc.robot.Commands.Intake.IntakePrepareToIndex;
import frc.robot.Commands.Intake.IntakeRun;
import frc.robot.Commands.Intake.IntakeStop;
import frc.robot.Commands.Shooter.ShootAtFarshot;
import frc.robot.Commands.Shooter.ShootAtPlatform;
import frc.robot.Commands.Shooter.ShootAtSubwoofer;
import frc.robot.Commands.Shooter.ShootInAmp;
import frc.robot.Commands.Shooter.ShootWithLimelight;
import frc.robot.Commands.Shooter.ShooterPrepareToIndex;
import frc.robot.Commands.Shooter.ShooterShoot;
import frc.robot.Commands.Shooter.StopShooter;
import frc.robot.Constants.Intake;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.LimelightWithBotPose;
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
  CANcoder leftBackCANcoder = new CANcoder(3, "1023");
  CANcoder rightBackCANcoder = new CANcoder(2, "1023");

  private double MaxSpeed = 3; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandXboxController DriverController = new CommandXboxController(0);
  XboxController Conttroller = new XboxController(1);
  // private final XboxController ManipulatorController = new XboxController(0);
  private final CommandXboxController ManipulatorController = new CommandXboxController(1);// My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // drivetrain
  Limelight limelightSubsystem = new Limelight();
  LimelightWithBotPose limelightWithBotPose = new LimelightWithBotPose();
  IntakeSubsystem IntakeSubsystem = new IntakeSubsystem();
  ShooterSubsystem ShooterSubsystem = new ShooterSubsystem();
  IndexerSubsystem IndexerSubsystem = new IndexerSubsystem();
  ClimberSubsystem ClimberSubsystem = new ClimberSubsystem();
  private final FieldCentric drive = new FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * .03) // Add a 15% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric, driving in open loop
  ShootAtSubwoofer shooterAtAmplifier = new ShootAtSubwoofer(ShooterSubsystem);
  IntakeNote intakeNote = new IntakeNote(IntakeSubsystem);
  IntakeStop intakeStop = new IntakeStop(IntakeSubsystem);
  ShooterPrepareToIndex shooterPrepareToIndex = new ShooterPrepareToIndex(ShooterSubsystem);
  IntakePrepareToIndex intakePrepareToIndex = new IntakePrepareToIndex(IntakeSubsystem);
  ShooterShoot shooterShoot = new ShooterShoot(ShooterSubsystem);
  StopShooter stopShooter = new StopShooter(ShooterSubsystem);
  FeedShooter feedShooter = new FeedShooter(IndexerSubsystem);
  StopIndex stopIndex = new StopIndex(IndexerSubsystem);
  ReverseIndexer reverseIndexer = new ReverseIndexer(IndexerSubsystem);
  IntakeDown intakeDown = new IntakeDown(IntakeSubsystem);
  IntakeRun intakeRun = new IntakeRun(IntakeSubsystem);
  ShootAtSubwoofer shootAtSubwoofer = new ShootAtSubwoofer(ShooterSubsystem);
  ShootAtPlatform shootAtPlatform = new ShootAtPlatform(ShooterSubsystem);
  ShootInAmp shootInAmp = new ShootInAmp(ShooterSubsystem);
  ClimberUp climberUp = new ClimberUp(ClimberSubsystem);
  ClimberDown climberDown = new ClimberDown(ClimberSubsystem);
  NotePassOff notePassOff = new NotePassOff(IntakeSubsystem, ShooterSubsystem, IndexerSubsystem);
  ClimberMaintainDown climberMaintainDown = new ClimberMaintainDown(ClimberSubsystem);
  ShootAtFarshot shootAtFarshot = new ShootAtFarshot(ShooterSubsystem);
  FeedShooterFast FeedShooterFast = new FeedShooterFast(IndexerSubsystem);
  ShootWithLimelight shootWithLimelight = new ShootWithLimelight(ShooterSubsystem, limelightSubsystem);
  private final SwerveDriveBrake brake = new SwerveDriveBrake();
  private final PointWheelsAt point = new PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  
  // private final DigitalInput indexerBeamBreak = new DigitalInput(0);

  public RobotContainer() {
    // ShooterSubsystem.setDefaultCommand(shooterPrepareToIndex);
    // IntakeSubsystem.setDefaultCommand(intakePrepareToIndex);
    // IndexerSubsystem.setDefaultCommand(notePassOff);
    // ShooterSubsystem.setDefaultCommand(notePassOff);
    // IntakeSubsystem.setDefaultCommand(notePassOff);
    // ClimberSubsystem.setDefaultCommand(climberMaintainDown);
 
    ManipulatorController.a()
        .whileTrue(intakeNote).whileFalse(intakeStop);
    ManipulatorController.b()
        .whileTrue(shootAtFarshot)
        .whileFalse(shooterPrepareToIndex);
    ManipulatorController.leftBumper()
         .whileTrue(FeedShooterFast).whileFalse(stopIndex);
    ManipulatorController.rightBumper()
         .whileTrue(reverseIndexer).whileFalse(stopIndex);
    ManipulatorController.pov(180)
        .whileTrue(intakeDown)
        .whileFalse(intakePrepareToIndex);
    ManipulatorController.pov(0)
        .whileTrue(intakeRun)
        .whileFalse(intakePrepareToIndex);
    ManipulatorController.pov(90)
        .whileTrue(shootAtSubwoofer)
        .whileFalse(shooterPrepareToIndex);
    ManipulatorController.pov(270)
        .whileTrue(notePassOff);
    ManipulatorController.y()
        .whileTrue(shootInAmp)
        .whileFalse(shooterPrepareToIndex);
    ManipulatorController.x()
       .whileTrue(shootWithLimelight).whileFalse(shooterPrepareToIndex);
    // ManipulatorController.leftBumper().whileTrue(ShooterSubsystem.PointTowardsSpeaker()).whileFalse(ShooterSubsystem.ShooterPrepareToIndex());
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-LeftYAxis * MaxSpeed) // Drive forward with negative Y
                                                                                 // (forward)
            .withVelocityY(-LeftXAxis * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-RightXAxis * MaxAngularRate)// RightXAxis * MaxAngularRate) // Drive counterclockwise
                                                             // with negative X (left)
        ));
    DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    DriverController.b().whileTrue(drivetrain
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-DriverController.getLeftY(), -DriverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    DriverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  @Override
  public void periodic() {
    if ((DriverController.getRightX() > .15) || (DriverController.getRightX() < -.15)) {
      RightXAxis = DriverController.getRightX();
    } else if (Conttroller.getXButton()) {
      RightXAxis = -limelightSubsystem.rotationtmp;
    } else {
      RightXAxis = 0;
    }
    if ((DriverController.getLeftY() > .15) || (DriverController.getLeftY() < -.15)) {
      LeftYAxis = DriverController.getLeftY();
    } else {
      LeftYAxis = 0;
    }
    if ((DriverController.getLeftX() > .15) || (DriverController.getLeftX() < -.15)) {
      LeftXAxis = DriverController.getLeftX();
    } else {
      LeftXAxis = 0;
    }

    SmartDashboard.putNumber("LeftxAxis", LeftXAxis);
    SmartDashboard.putNumber("LeftYAxis", LeftYAxis);
    SmartDashboard.putNumber("RightxAxis", RightXAxis);
    SmartDashboard.putNumber("RightYaxis", RightYAxis);

    shooterCurrentRPMValue = ShooterSubsystem.shooterCurrentRPM;
    
    SmartDashboard.putNumber("left front CANcoder", leftFrontCANcoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("right front CANcoder", rightFrontCANcoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("left back CANcoder", leftBackCANcoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("right back CANcoder", rightBackCANcoder.getAbsolutePosition().getValueAsDouble());
  }
}
