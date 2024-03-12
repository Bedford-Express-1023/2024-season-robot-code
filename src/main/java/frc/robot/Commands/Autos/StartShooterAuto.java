// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;


public class StartShooterAuto extends Command{
  /** Creates a new ShootBack. */
    ShooterSubsystem s_ShooterSubsystem;
    
  long shooterStartTime;
  public StartShooterAuto(ShooterSubsystem s_ShooterSubsystem) {
this.s_ShooterSubsystem = s_ShooterSubsystem;
addRequirements(s_ShooterSubsystem);
shooterStartTime = -1;
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    shooterStartTime = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ShooterSubsystem.ShooterShoot();
    if (shooterStartTime == -1) {

      shooterStartTime = System.currentTimeMillis();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterStartTime != -1 && (System.currentTimeMillis() - shooterStartTime) > 1000;
  }
}
