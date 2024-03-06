// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class OutTake extends Command {
  IntakeSubsystem s_IntakeSubsystem;
  /** Creates a new OutTake. */
  public OutTake( IntakeSubsystem s_IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    addRequirements(s_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_IntakeSubsystem.OutTake();
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
