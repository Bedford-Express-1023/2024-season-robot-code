// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.TunerConstants;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeLimelight;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class DriveAtNoteAuto extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    IntakeLimelight s_IntakeLimelight;
    IntakeSubsystem s_IntakeSubsystem;
  ShooterSubsystem s_ShooterSubsystem;
  IndexerSubsystem s_IndexerSubsystem; 
  int counter;
  private final RobotCentric driveRobotCentric = new RobotCentric();
  /** Creates a new DriveAtNote. */
  public DriveAtNoteAuto(CommandSwerveDrivetrain drivetrain, IntakeLimelight s_IntakeLimelight,
   IntakeSubsystem s_IntakeSubsystem, ShooterSubsystem s_ShooterSubsystem, IndexerSubsystem s_IndexerSubsystem) {
    this.s_IntakeLimelight = s_IntakeLimelight;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.s_IndexerSubsystem = s_IndexerSubsystem;
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain, s_IntakeLimelight, s_IntakeSubsystem, s_IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_IntakeLimelight.resetPID();
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // drivetrain.applyRequest(()-> driveRobotCentric.withVelocityX(.1). withVelocityY(.1));
   s_IntakeLimelight.SetPid();
   if(s_IntakeLimelight.NoteSeen())
 {
drivetrain.driveRobotRelative(new ChassisSpeeds(0,0,s_IntakeLimelight.intakeRotation *3));
    if (s_IntakeLimelight.PointedAtNote() == true){
drivetrain.driveRobotRelative(new ChassisSpeeds(1.5,0,s_IntakeLimelight.intakeRotation *3));
s_ShooterSubsystem.ShooterPrepareToIndex();
   s_IntakeSubsystem.IntakeRun();
    }
    }
    if(s_IntakeSubsystem.intakeBeamBreakValue == false){
      s_IntakeSubsystem.IntakePrepareToIndex();
    }

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (s_IntakeSubsystem.intakeBeamBreakValue == false){
    return true;
   }
   else{
    return false;
   }
  }
}
