// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_CAN);
  private final TalonFX leftShooterPivotMotor = new TalonFX(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN);
  private final TalonFX rightShooterPivotMotor = new TalonFX(Constants.Shooter.SHOOTER_RIGHT_PIVOT_CAN);

  private double leftShooterPivotMotorAngle; //in degrees
  private double rightShooterPivotMotorAngle; //in degrees
  public boolean shooterReadyToIndex;

  private double shooterCurrentVelocity;
  private double shooterTargetVelocity;

  public Slot0Configs slot0Configs = new Slot0Configs();
  public VelocityVoltage shooterVelocityAmplifier = new VelocityVoltage(Constants.Shooter.shooterVelocityAmplifierConstant, 0, false, 0, 0, false, false, false);
  public VelocityVoltage shooterVelocityPlatform = new VelocityVoltage(0, Constants.Shooter.shooterVelocityPlatformConstant, false, 0, 0, false, false, false);
  public VelocityVoltage shooterVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    slot0Configs.kV = 1;
    slot0Configs.kP = 0.1;
    slot0Configs.kI = 0.05;
    slot0Configs.kD = 0.01;
    shooterMotor.getConfigurator().apply(slot0Configs, 0.050);

  }

  public void ShootAtAmplifier() {
    shooterMotor.setControl(shooterVelocityAmplifier);
  }

  public void ShootAtPlatform() {
    shooterMotor.setControl(shooterVelocityPlatform);
  }

  public void ShootWithLimelight() {
    shooterTargetVelocity = 0; //FIXME insert interpolation equation here
    
    shooterMotor.setControl(new VelocityVoltage(shooterTargetVelocity, 0, false, 0, 0, false, false, false));
  }

  public void StopShooter() {
    shooterMotor.set(0);
  }

  public void ShooterPrepareToIndex() {
    leftShooterPivotMotor.setPosition(Constants.Shooter.targetShooterPivotIndexAngle);
    rightShooterPivotMotor.setPosition(Constants.Shooter.targetShooterPivotIndexAngle);
  }

  public void PointTowardsSpeaker() {
    
  }

  @Override
  public void periodic() {
    shooterCurrentVelocity = shooterMotor.getVelocity().getValueAsDouble();

    leftShooterPivotMotorAngle = (leftShooterPivotMotor.getPosition().getValueAsDouble() * 360);
    rightShooterPivotMotorAngle = (rightShooterPivotMotor.getPosition().getValueAsDouble() * 360);

    if ((leftShooterPivotMotorAngle > Constants.Shooter.minShooterPivotIndexAngle) && (leftShooterPivotMotorAngle < Constants.Shooter.maxShooterPivotIndexAngle)) {
      shooterReadyToIndex = true;
    }
    else {
      shooterReadyToIndex = false;
    }

    // This method will be called once per scheduler run
  }
}
