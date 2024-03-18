// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Subsystems.Limelight;

public class PointAtSpeaker extends Command {
  private final CommandSwerveDrivetrain drivetrain;
 private final Limelight s_limelightSubsystem;
  /** Creates a new SwerveXPattern. */
  public PointAtSpeaker(CommandSwerveDrivetrain drivetrain, Limelight s_limelightSubsystem ) {
        this.drivetrain = drivetrain;
        this.s_limelightSubsystem = s_limelightSubsystem;
        addRequirements(drivetrain, s_limelightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_limelightSubsystem.RotateWithLimelight();
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, s_limelightSubsystem.rotationtmp * 2 *Math.PI));
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
    return false;
  }
}
