// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class IntakeToPassOff extends Command {
  IntakeSubsystem s_IntakeSubsystem;
  ShooterSubsystem s_ShooterSubsystem;
  IndexerSubsystem s_IndexerSubsystem;
  int counter;
  /** Creates a new IntakeToPassOff. */
  public IntakeToPassOff(IntakeSubsystem s_IntakeSubsystem, ShooterSubsystem s_ShooterSubsystem, IndexerSubsystem s_IndexerSubsystem) {
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.s_IndexerSubsystem = s_IndexerSubsystem;
    addRequirements(s_IntakeSubsystem, s_ShooterSubsystem, s_IndexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ShooterSubsystem.shooterPivotPID.reset();
    s_IntakeSubsystem.IntakePivotPID.reset();
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ShooterSubsystem.ShooterPrepareToIndex();
    if (s_IntakeSubsystem.intakeBeamBreakValue == true && counter == 0) {
      s_IntakeSubsystem.IntakeNote();
      s_IntakeSubsystem.IntakeDown();
    } else {
      counter = 1; 
      s_IntakeSubsystem.IntakePrepareToIndex();
      //s_ShooterSubsystem.ShooterPrepareToIndex();
        if ((s_ShooterSubsystem.shooterReadyToIndex == true)
         && (s_IntakeSubsystem.intakeReadyToIndex == true)
          && (s_IndexerSubsystem.indexerBeamBreakValue == true)
          && (counter == 1)
          ) {
          s_IntakeSubsystem.IntakeNote();
          s_IndexerSubsystem.IndexNote();
        } else {
          s_IntakeSubsystem.IntakeStop();
          s_IndexerSubsystem.StopIndex();
        }
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_IntakeSubsystem.IntakeStop();
    s_IndexerSubsystem.StopIndex();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_IndexerSubsystem.indexerBeamBreakValue == false;
  }
}
