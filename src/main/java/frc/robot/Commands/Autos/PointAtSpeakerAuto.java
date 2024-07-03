// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Subsystems.ShooterLimelight;

public class PointAtSpeakerAuto extends Command {
  private final CommandSwerveDrivetrain drivetrain;
 private final ShooterLimelight s_limelightSubsystem;
  /** Creates a new SwerveXPattern. */
  public PointAtSpeakerAuto(CommandSwerveDrivetrain drivetrain, ShooterLimelight s_limelightSubsystem ) {
        this.drivetrain = drivetrain;
        this.s_limelightSubsystem = s_limelightSubsystem;
        addRequirements(drivetrain, s_limelightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_limelightSubsystem.pidRotation.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_limelightSubsystem.RotateWithLimelight();
        drivetrain.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, s_limelightSubsystem.rotationtmp * 8));
  
      if(s_limelightSubsystem.IsRotated())
    {
      drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_limelightSubsystem.StopRotatingWithLimelight();
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(s_limelightSubsystem.IsRotated() && s_limelightSubsystem.AprilTagSeen())
    {
      return true;
    }
    else{
    return false;
    }
  }
}
