// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.TunerConstants;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeLimelight;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class DriveAtNote extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    IntakeLimelight s_IntakeLimelight;
    ShooterSubsystem s_ShooterSubsystem;
    IntakeSubsystem s_IntakeSubsystem;
    IndexerSubsystem s_IndexerSubsystem;
  private final RobotCentric driveRobotCentric = new RobotCentric();
  /** Creates a new DriveAtNote. */
  public DriveAtNote(CommandSwerveDrivetrain drivetrain, IntakeLimelight s_IntakeLimelight,
   ShooterSubsystem s_ShooterSubsystem, IntakeSubsystem s_IntakeSubsystem, IndexerSubsystem s_IndexerSubsystem) {
    this.s_IntakeLimelight = s_IntakeLimelight;
    this.s_IntakeSubsystem = s_IntakeSubsystem;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.s_IndexerSubsystem = s_IndexerSubsystem;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain, s_IntakeLimelight, s_ShooterSubsystem, s_IntakeSubsystem, s_IndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_IntakeLimelight.resetPID();
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
 drivetrain.driveRobotRelative(new ChassisSpeeds(.5,0,s_IntakeLimelight.intakeRotation *3));
 s_ShooterSubsystem.ShooterPrepareToIndex();
    s_IntakeSubsystem.IntakeRun();
     }
     }

      if (s_IntakeSubsystem.intakeBeamBreakValue == true) {

 } else {
   s_IntakeSubsystem.IntakePrepareToIndex(); 
    drivetrain.driveRobotRelative(new ChassisSpeeds(0,0,0));
   //s_ShooterSubsystem.ShooterPrepareToIndex();
     if ((s_ShooterSubsystem.shooterReadyToIndex == true)
      && (s_IntakeSubsystem.intakeReadyToIndex == true)
       && (s_IndexerSubsystem.indexerBeamBreakValue == true)
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
s_IntakeSubsystem.IntakePrepareToIndex();
 drivetrain.driveRobotRelative(new ChassisSpeeds(0,0,0));
 }
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
