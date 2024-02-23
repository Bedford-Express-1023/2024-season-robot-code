// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShootInAmp extends Command {
  ShooterSubsystem s_ShooterSubsystem;
  /** Creates a new ShootInAmp. */
  public ShootInAmp(ShooterSubsystem s_ShooterSubsystem) {
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    addRequirements(s_ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ShooterSubsystem.ShootInAmp();
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
