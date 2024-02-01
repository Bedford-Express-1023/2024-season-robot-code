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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Swerve;

public class RobotContainer extends SubsystemBase {
  public static double LeftXAxis;
  public static double LeftYAxis;
  public static double RightXAxis;
  public static double RightYAxis;
  
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final Swerve drivetrain = TunerConstants.DriveTrain; //  drivetrain

  private final FieldCentric drive = new FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 15% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric, driving in open loop
  private final SwerveDriveBrake brake = new SwerveDriveBrake();
  private final PointWheelsAt point = new PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-LeftYAxis * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-LeftXAxis * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-RightXAxis * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-LeftYAxis, -LeftXAxis))));
    configureBindings();
  }

  private void configureBindings() {
    
    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  @Override
  public void periodic() {
    LeftXAxis = joystick.getLeftX();
    LeftYAxis = joystick.getLeftY();
    RightXAxis = joystick.getRightX();
    RightYAxis = joystick.getRightY();
    
    SmartDashboard.putNumber("LeftxAxis", LeftXAxis);
    SmartDashboard.putNumber("LeftYAxis", LeftYAxis);
    SmartDashboard.putNumber("RightxAxis", RightXAxis);
    SmartDashboard.putNumber("RightYaxis", RightYAxis);
  }
}