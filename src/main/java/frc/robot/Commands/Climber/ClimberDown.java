// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class ClimberDown extends Command {
  ClimberSubsystem s_ClimberSubsystem;
  ShooterSubsystem s_ShooterSubsystem;
  /** Creates a new ClimberDown. */
  public ClimberDown(ClimberSubsystem s_ClimberSubsystem, ShooterSubsystem s_ShooterSubsystem) {
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.s_ClimberSubsystem = s_ClimberSubsystem;
    addRequirements(s_ClimberSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ClimberSubsystem.ClimberDown();
   // s_ShooterSubsystem.ShooterDown();
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
