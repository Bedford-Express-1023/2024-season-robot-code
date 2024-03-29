// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IndexerSubsystem;

public class FeedShooter extends Command {
  IndexerSubsystem s_IndexerSubsystem;
  /** Creates a new FeedShooter. */
  public FeedShooter(IndexerSubsystem s_IndexerSubsystem) {
    this.s_IndexerSubsystem = s_IndexerSubsystem;
    addRequirements(s_IndexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_IndexerSubsystem.FeedShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_IndexerSubsystem.StopIndex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
