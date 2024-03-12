// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class SwerveXPattern extends Command {
  private final SwerveDrivetrain drivetrain;
  /** Creates a new SwerveXPattern. */
  public SwerveXPattern(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
     //   addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.CommandVariable = "Swerve X";
    //         drivetrain.frontLeftModule.set(0, (Constants.FRONT_LEFT_MODULE_STEER_OFFSET     - 135));
    //         drivetrain.frontRightModule.set(0, (Constants.FRONT_RIGHT_MODULE_STEER_OFFSET   + 135));
    //         drivetrain.backLeftModule.set(0, (Constants.BACK_LEFT_MODULE_DRIVE_MOTOR        - 45));
    //         drivetrain.backRightModule.set(0, (Constants.BACK_RIGHT_MODULE_STEER_OFFSET     + 45));
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
