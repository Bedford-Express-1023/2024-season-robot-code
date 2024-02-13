// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LinearInterpolator;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_CAN);
  private final TalonFX shooterPivotMotorMaster = new TalonFX(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN); //left pivot motor
  private final TalonFX shooterPivotMotorFollower = new TalonFX(Constants.Shooter.SHOOTER_RIGHT_PIVOT_CAN); //right pivot motor

  private final Follower pivotFollower = new Follower(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN, true);
  //public static final LinearInterpolator shooterInterpolator = new LinearInterpolator(Constants.Shooter.shooterTable);
  //public static final LinearInterpolator shooterPivotInterpolator = new LinearInterpolator(Constants.Shooter.pivotTable);

  private double shooterCurrentAngle; //in degrees
  private double shooterTargetAngle; //in degrees
  private double shooterCurrentRPM;
  private double shooterTargetRPM;
  public boolean shooterReadyToIndex;

  //Tester variables
  public double amplifierTargetRPM;
  public double amplifierTargetAngle;
  public double shooterP;
  public double shooterI;
  public double shooterD;
  public double pivotP;
  public double pivotI;
  public double pivotD;

  public Slot0Configs slot0Configs = new Slot0Configs();
  public Slot1Configs slot1Configs = new Slot1Configs();
  public VelocityVoltage shooterVelocityAmplifier = new VelocityVoltage(SmartDashboard.getNumber("amplifier target RPM", amplifierTargetRPM)/*Constants.Shooter.shooterVelocityAmplifierConstant*/, 0, false, 0, 0, false, false, false); //FIXME temporary change for testing
  public VelocityVoltage shooterAngleAmplifier = new VelocityVoltage(SmartDashboard.getNumber("amplifier target angle", amplifierTargetAngle)/*Constants.Shooter.shooterAngleAmplifierConstant*/, 0, false, 0, 0, false, false, false); //FIXME temporary change for testing
  public VelocityVoltage shooterVelocityPlatform = new VelocityVoltage(Constants.Shooter.shooterVelocityPlatformConstant, 0, false, 0, 0, false, false, false);
  public VelocityVoltage shooterAnglePlatform = new VelocityVoltage(Constants.Shooter.shooterAnglePlatformConstant, 0, false, 0, 0, false, false, false);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    slot0Configs.kV = 1; //shooter config velocity feedforward gain
    slot0Configs.kP = SmartDashboard.getNumber("shooter P value", 0); //shooter config proportional gain
    slot0Configs.kI = SmartDashboard.getNumber("shooter I value", 0); //shooter config integral gain
    slot0Configs.kD = SmartDashboard.getNumber("shooter D value", 0); //shooter config derivative gain

    slot1Configs.kV = 1; //pivot config velocity feedforward gain
    slot1Configs.kP = SmartDashboard.getNumber("shooter P value", 0); //pivot config proportional gain, originally 0.1
    slot1Configs.kI = SmartDashboard.getNumber("shooter I value", 0); //pivot config integral gain, originally 0.05
    slot1Configs.kD = SmartDashboard.getNumber("shooter D value", 0); //pivot config derivative gain, originally 0.01

    shooterMotor.getConfigurator().apply(slot0Configs, 0.050); 
    shooterPivotMotorMaster.getConfigurator().apply(slot1Configs, 0.050); 
    shooterPivotMotorFollower.getConfigurator().apply(slot1Configs, 0.050); 
    shooterPivotMotorMaster.setControl(pivotFollower); //config right pivot motor follows left pivot motor
  }
 
  public void ShootAtAmplifier() {
    shooterMotor.setControl(shooterVelocityAmplifier);
    shooterMotor.setControl(shooterAngleAmplifier);
  }

  public void ShootAtPlatform() {
    shooterMotor.setControl(shooterVelocityPlatform);
    shooterMotor.setControl(shooterAnglePlatform);
  }

  public void ShootWithLimelight() {
    shooterMotor.setControl(new VelocityVoltage((shooterTargetRPM * 60), 0, false, 0, 0, false, false, false));
    shooterPivotMotorMaster.setControl(new PositionVoltage(shooterTargetAngle, 0, false, 0, 1, false, false, false));
  }

  public void StopShooter() {
    shooterMotor.set(0);
  }

  public void ShooterPrepareToIndex() {
    shooterPivotMotorMaster.setPosition(Constants.Shooter.targetShooterPivotIndexAngle);
    shooterPivotMotorFollower.setPosition(Constants.Shooter.targetShooterPivotIndexAngle);
  }

  public void PointTowardsSpeaker() {
    
  }

  @Override
  public void periodic() {
    shooterCurrentRPM = (shooterMotor.getVelocity().getValueAsDouble() / 60);
    shooterCurrentAngle = (shooterPivotMotorMaster.getPosition().getValueAsDouble() * 360);

    if ((shooterCurrentAngle > Constants.Shooter.minShooterPivotIndexAngle) && (shooterCurrentAngle < Constants.Shooter.maxShooterPivotIndexAngle)) {
      shooterReadyToIndex = true;
    }
    else {
      shooterReadyToIndex = false;
    }

    //shooterTargetRPM = shooterInterpolator.getInterpolatedValue(0); //FIXME insert current limelight y axis data
    //shooterTargetAngle = shooterPivotInterpolator.getInterpolatedValue(0); //FIXME insert current limelight y axis data

    SmartDashboard.putNumber("amplifier target RPM", amplifierTargetRPM);
    SmartDashboard.putNumber("amplifier target angle", amplifierTargetAngle);
    SmartDashboard.putNumber("shooter P value", shooterP);
    SmartDashboard.putNumber("shooter I value", shooterI);
    SmartDashboard.putNumber("shooter D value", shooterD);
    SmartDashboard.putNumber("pivot P value", pivotP);
    SmartDashboard.putNumber("pivot I value", pivotI);
    SmartDashboard.putNumber("pivot D value", pivotD);

    SmartDashboard.putNumber("current shooter RPM", shooterCurrentRPM);
    SmartDashboard.putNumber("target RPM with limelight", shooterTargetRPM);
    SmartDashboard.putNumber("current shooter angle", shooterCurrentAngle);
    SmartDashboard.putNumber("target angle with limelight", shooterTargetAngle);

    // This method will be called once per scheduler run
  }
}
