// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TrapDoor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class FirstTrapdoorSpot extends Command {
  ShooterSubsystem s_ShooterSubsystem;
  ClimberSubsystem s_ClimberSubsystem;
  IntakeSubsystem s_IntakeSubsystem;
  /** Creates a new ShootInAmp. */
  public FirstTrapdoorSpot(ShooterSubsystem s_ShooterSubsystem, ClimberSubsystem s_ClimberSubsystem, IntakeSubsystem s_IntakeSubsystem) {
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.s_ClimberSubsystem = s_ClimberSubsystem;
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    addRequirements(s_ShooterSubsystem, s_ClimberSubsystem, s_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ShooterSubsystem.shooterPivotPID.reset();
    s_IntakeSubsystem.IntakePivotPID.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ShooterSubsystem.ShooterToFirstClimb();
    s_ClimberSubsystem.ClimberUpWithSwitch();
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
