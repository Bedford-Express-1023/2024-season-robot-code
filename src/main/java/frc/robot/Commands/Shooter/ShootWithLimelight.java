// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShootWithLimelight extends Command {
  ShooterSubsystem s_ShooterSubsystem;
  Limelight s_Limelight;
  /** Creates a new ShootWithLimelight. */
  public ShootWithLimelight(ShooterSubsystem s_ShooterSubsystem, Limelight s_Limelight) {
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.s_Limelight = s_Limelight;
    addRequirements(s_ShooterSubsystem, s_Limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ShooterSubsystem.shooterPivotPID.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ShooterSubsystem.ShootWithLimelight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
