// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class NotePassOff extends Command {
  IntakeSubsystem s_IntakeSubsystem;
  ShooterSubsystem s_ShooterSubsystem;
  IndexerSubsystem s_IndexerSubsystem;
  boolean betweenBeamBreaksBoolean;

  public NotePassOff(IntakeSubsystem s_IntakeSubsystem, ShooterSubsystem s_ShooterSubsystem, IndexerSubsystem s_IndexerSubsystem) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putBoolean("NotePassOff is running", true);
    if((s_IntakeSubsystem.PivotCANCoder.getAbsolutePosition().getValueAsDouble() > Constants.Intake.intakeDownPosition - 0.03) || 
      (s_IntakeSubsystem.PivotCANCoder.getAbsolutePosition().getValueAsDouble() < Constants.Intake.intakeDownPosition + 0.03)) 
    {
      s_IntakeSubsystem.IntakePrepareToIndex();
      s_ShooterSubsystem.ShooterPrepareToIndex();
      //SmartDashboard.putBoolean("NotePassOff is going to indexing angles", true);

      if (s_IntakeSubsystem.intakeBeamBreakValue == false) {
        s_IntakeSubsystem.IntakeStop();
        SmartDashboard.putBoolean("Intake Bream Break false- Intake Stop", true);

        if ((s_ShooterSubsystem.shooterReadyToIndex == true) && (s_IntakeSubsystem.intakeReadyToIndex == true)) {
          s_IntakeSubsystem.IntakeNote();
          s_IndexerSubsystem.IndexNote();
          SmartDashboard.putBoolean("NotePassOff is at indexing angles", true);
        } else {
          s_IntakeSubsystem.IntakeStop();
          s_IndexerSubsystem.StopIndex();
        }
      } else if (s_IntakeSubsystem.intakeBeamBreakValue == true) {
        //s_IntakeSubsystem.IntakeStop();

        if (s_IndexerSubsystem.indexerBeamBreakValue == false) {
          SmartDashboard.putBoolean("NotePassOff ", true);
          s_IndexerSubsystem.StopIndex();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
