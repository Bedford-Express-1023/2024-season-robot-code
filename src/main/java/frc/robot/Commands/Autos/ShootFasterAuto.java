// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.ShooterSubsystem;

public class ShootFasterAuto extends Command {
  ShooterSubsystem s_ShooterSubsystem;
  Limelight s_Limelight;
  IndexerSubsystem s_IndexerSubsystem;
  long shooterStartTime;

  /** Creates a new ShootWithLimelight. */
  public ShootFasterAuto(ShooterSubsystem s_ShooterSubsystem, Limelight s_Limelight,
      IndexerSubsystem s_IndexerSubsystem) {
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.s_Limelight = s_Limelight;
    this.s_IndexerSubsystem = s_IndexerSubsystem;
    addRequirements(s_ShooterSubsystem, s_Limelight, s_IndexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        System.out.println("Shoot starting");

    s_ShooterSubsystem.shooterPivotPID.reset();

    shooterStartTime = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Shooting1");
    s_ShooterSubsystem.ShootWithLimelight();
    if (s_ShooterSubsystem.ReadyToShootAutofaster()) {
      System.out.println("Shooting2");
      s_IndexerSubsystem.FeedShooterFast();
      if (shooterStartTime == -1) {
        System.out.println("Shooting3");

        shooterStartTime = System.currentTimeMillis();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Shooting end");

    s_ShooterSubsystem.shooterMotor.set(0);
    s_IndexerSubsystem.indexerMotor.set(ControlMode.PercentOutput, 0);
    s_ShooterSubsystem.shooterPivotMotorMaster.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterStartTime != -1 && (System.currentTimeMillis() - shooterStartTime) > 400;
  }
}

