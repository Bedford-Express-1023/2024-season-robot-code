// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class NotePassOffAuto extends Command {
  IntakeSubsystem s_IntakeSubsystem;
  ShooterSubsystem s_ShooterSubsystem;
  IndexerSubsystem s_IndexerSubsystem;
  boolean betweenBeamBreaksBoolean;

  public NotePassOffAuto(IntakeSubsystem s_IntakeSubsystem, ShooterSubsystem s_ShooterSubsystem, IndexerSubsystem s_IndexerSubsystem) {
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.s_IndexerSubsystem = s_IndexerSubsystem;
    addRequirements(s_IntakeSubsystem, s_ShooterSubsystem, s_IndexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting NotePassoff");

    s_ShooterSubsystem.shooterPivotPID.reset();
    s_IntakeSubsystem.IntakePivotPID.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Running Passoff");

    //SmartDashboard.putBoolean("NotePassOff is running", true);
    if((s_IntakeSubsystem.PivotCANCoder.getAbsolutePosition().getValueAsDouble() > Constants.Intake.intakeDownPosition - 0.03) ||
      (s_IntakeSubsystem.PivotCANCoder.getAbsolutePosition().getValueAsDouble() < Constants.Intake.intakeDownPosition + 0.03))
    {
          System.out.println("prepare to index Passoff");

      s_IntakeSubsystem.IntakePrepareToIndex();
      s_ShooterSubsystem.ShooterPrepareToIndex();
      //SmartDashboard.putBoolean("NotePassOff is going to indexing angles", true);

     

        if ((s_ShooterSubsystem.shooterReadyToIndex == true) && (s_IntakeSubsystem.intakeReadyToIndex == true)) {
              System.out.println("transitioning Passoff");

          s_IntakeSubsystem.IntakeNote();
          s_IndexerSubsystem.IndexNote();
          SmartDashboard.putBoolean("NotePassOff is at indexing angles", true);
        } else {
              System.out.println("waiting for transition Passoff");

          s_IntakeSubsystem.IntakeStop();
          s_IndexerSubsystem.StopIndex();
        }
      }  if (s_IndexerSubsystem.indexerBeamBreakValue == false) {
              System.out.println("indexer beam break Passoff");

          SmartDashboard.putBoolean("NotePassedOff ", true);
          s_IndexerSubsystem.StopIndex();
          s_IntakeSubsystem.IntakeStop();
        }
    }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
                  System.out.println("ending Passoff");

    s_IndexerSubsystem.indexerMotor.set(ControlMode.PercentOutput, 0);
    s_IntakeSubsystem.intakeMotor.set(0);
    s_IntakeSubsystem.intakePivotMotor.set(0);
    s_ShooterSubsystem.shooterMotor.set(0);
    s_ShooterSubsystem.shooterPivotMotorMaster.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
                  System.out.println("indexer beam break Passoff is " + s_IndexerSubsystem.indexerBeamBreakValue);

    return !s_IndexerSubsystem.indexerBeamBreakValue;
  }
}