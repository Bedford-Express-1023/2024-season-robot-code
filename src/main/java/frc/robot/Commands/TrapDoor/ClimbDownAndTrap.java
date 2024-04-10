// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TrapDoor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class ClimbDownAndTrap extends Command {
  ClimberSubsystem s_ClimberSubsystem;
  IndexerSubsystem s_IndexerSubsystem;
  IntakeSubsystem s_IntakeSubsystem;
  /** Creates a new ClimberDown. */
  public ClimbDownAndTrap(ClimberSubsystem s_ClimberSubsystem, IndexerSubsystem s_IndexerSubsystem, IntakeSubsystem s_IntakeSubsystem) {
    this.s_ClimberSubsystem = s_ClimberSubsystem;
    this.s_IndexerSubsystem = s_IndexerSubsystem;
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    addRequirements(s_ClimberSubsystem, s_IndexerSubsystem, s_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ClimberSubsystem.ClimberDownWithSwitch();
    s_IntakeSubsystem.IntakeDown();
 if (s_ClimberSubsystem.ClimberOnLimitSwitch()){
  s_IndexerSubsystem.ReverseIndexerTrapdoor();
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
