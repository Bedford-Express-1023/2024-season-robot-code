// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShootBackAuto extends Command {
  /** Creates a new ShootBack. */
    ShooterSubsystem s_ShooterSubsystem;
  IndexerSubsystem s_IndexerSubsystem;
  long shooterStartTime;
  public ShootBackAuto(ShooterSubsystem s_ShooterSubsystem, IndexerSubsystem s_IndexerSubsystem) {
this.s_ShooterSubsystem = s_ShooterSubsystem;
this.s_IndexerSubsystem = s_IndexerSubsystem;
addRequirements(s_ShooterSubsystem, s_IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ShooterSubsystem.shooterPivotPID.reset();
    
    shooterStartTime = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ShooterSubsystem.ShooterPrepareToIndex();
    s_ShooterSubsystem.ShooterShoot();
    if(s_ShooterSubsystem.ShooterShootIsReady())
    {
      s_IndexerSubsystem.IndexNote();
      if (shooterStartTime == -1) {
        shooterStartTime = System.currentTimeMillis();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    s_ShooterSubsystem.shooterMotor.set(0);
    s_IndexerSubsystem.indexerMotor.set(ControlMode.PercentOutput, 0);
    s_ShooterSubsystem.shooterPivotMotorMaster.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  shooterStartTime != -1 && (System.currentTimeMillis() - shooterStartTime) > 1000;
  }
}
