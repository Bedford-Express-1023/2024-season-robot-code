// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexNote extends Command {
  IndexerSubsystem s_IndexerSubsystem;
  ShooterSubsystem s_ShooterSubsystem;
  IntakeSubsystem s_IntakeSubsystem;
  boolean shooterReadyToIndex = s_ShooterSubsystem.shooterReadyToIndex;
  boolean intakeReadyToIndex = s_IntakeSubsystem.intakeReadyToIndex;

  /** Creates a new IndexNote. */
  public IndexNote() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((shooterReadyToIndex = true) && (intakeReadyToIndex)) {
    s_IndexerSubsystem.IndexNote();
    }
    else {
      s_ShooterSubsystem.ShooterPrepareToIndex();
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
