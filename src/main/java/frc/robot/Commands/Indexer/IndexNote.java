// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class IndexNote extends Command {
  IndexerSubsystem s_IndexerSubsystem;
  ShooterSubsystem s_ShooterSubsystem;
  IntakeSubsystem s_IntakeSubsystem;
  boolean shooterReadyToIndex = s_ShooterSubsystem.shooterReadyToIndex;
  boolean intakeReadyToIndex = s_IntakeSubsystem.intakeReadyToIndex;

  /** Creates a new IndexNote. */
  public IndexNote(IndexerSubsystem s_IndexerSubsystem, IntakeSubsystem s_IntakeSubsystem) {
    this.s_IndexerSubsystem = s_IndexerSubsystem;
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    addRequirements(s_IndexerSubsystem, s_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((shooterReadyToIndex == true) && (intakeReadyToIndex == true)) {
      s_IntakeSubsystem.IntakeNote();
      s_IndexerSubsystem.IndexNote();
      //s_IndexerSubsystem.IndexPause();
    }
    else {
      s_IntakeSubsystem.IntakeStop();
      s_IndexerSubsystem.StopIndex();
    }
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
