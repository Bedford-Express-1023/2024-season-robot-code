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
s_ShooterSubsystem.ShooterPrepareToIndex();
  if (s_IntakeSubsystem.intakeBeamBreakValue == true && counter == 0) {
    s_IntakeSubsystem.IntakeNote();
    s_IntakeSubsystem.IntakeDown();
        s_IntakeLimelight.SetPid();
 drivetrain.driveRobotRelative(new ChassisSpeeds(0,0,s_IntakeLimelight.intakeRotation *3));
     if (s_IntakeLimelight.PointedAtNote() == true){
  s_IntakeSubsystem.IntakeRun();
 drivetrain.driveRobotRelative(new ChassisSpeeds(2,0,s_IntakeLimelight.intakeRotation *3));
    
  }
  } else {
    counter = 1; 
    s_IntakeSubsystem.IntakePrepareToIndex(); 
     drivetrain.driveRobotRelative(new ChassisSpeeds(0,0,0));
    //s_ShooterSubsystem.ShooterPrepareToIndex();
      if ((s_ShooterSubsystem.shooterReadyToIndex == true)
       && (s_IntakeSubsystem.intakeReadyToIndex == true)
        && (s_IndexerSubsystem.indexerBeamBreakValue == true)
        && (counter == 1)
        ) {
        s_IntakeSubsystem.IntakeNote();
        s_IndexerSubsystem.FeedPassoff();
      } else {
        s_IntakeSubsystem.IntakeStop();
        s_IndexerSubsystem.StopIndex();
      }
  }
       if (s_IndexerSubsystem.indexerBeamBreakValue == false){
 s_IndexerSubsystem.StopIndex();
    s_IntakeSubsystem.IntakeStop();
  }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (s_IndexerSubsystem.indexerBeamBreakValue == false){
    return true;
   }
   else{
    return false;
   }
  }
}
