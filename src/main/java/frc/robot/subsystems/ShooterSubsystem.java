// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_CAN);
  private final TalonFX leftShooterPivotMotor = new TalonFX(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN);
  private final TalonFX rightShooterPivotMotor = new TalonFX(Constants.Shooter.SHOOTER_RIGHT_PIVOT_CAN);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
  }

  public void ShootAtAmplifier() {

  }

  public void ShootAtPlatform() {

  }

  public void StopShooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
