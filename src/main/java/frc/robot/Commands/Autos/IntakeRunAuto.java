// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeRunAuto extends Command {
  IntakeSubsystem s_IntakeSubsystem;

  /** Creates a new IntakeRun. */
  public IntakeRunAuto(IntakeSubsystem s_IntakeSubsystem) {
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    addRequirements(s_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting IntakeRun");
    s_IntakeSubsystem.IntakePivotPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        System.out.println(" IntakeRun");

    s_IntakeSubsystem.IntakeRun();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        System.out.println("Ending IntakeRun");

  s_IntakeSubsystem.intakePivotMotor.set(0);
  s_IntakeSubsystem.intakeMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !s_IntakeSubsystem.intakeBeamBreakValue;
  }
}