// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RotationalFeedForward;

public class ShooterSubsystem extends SubsystemBase {

  public final TalonFX shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_CAN);
  public final TalonFX shooterPivotMotorMaster = new TalonFX(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN);
  public final TalonFX shooterPivotMotorFollower = new TalonFX(Constants.Shooter.SHOOTER_RIGHT_PIVOT_CAN);
  private final CANcoder shooterCANcoder = new CANcoder(Constants.Shooter.SHOOTER_CANCODER_ID);

  private double shooterMotorAngle;
  public double shooterCurrentAngle; // in degrees
  public double shooterCurrentRPM;
  public double limelightTY;
  public boolean shooterReadyToIndex;
  public Slot0Configs slot0Configs = new Slot0Configs();
  public Slot1Configs slot1Configs = new Slot1Configs();

  public VelocityVoltage shooterVelocityFast = new VelocityVoltage(Constants.Shooter.shooterVelocitySubwooferConstant,
      0, false, 0, 0, false, false, false);
  public VelocityVoltage shooterVelocitySLow = new VelocityVoltage(Constants.Shooter.shooterVelocitySubwooferConstant,
      0, false, 0, 1, false, false, false);
  public PIDController shooterPivotPID = new PIDController(1.7, 0.38, 0);// (.85,0.075,0.0001);

  ArmFeedforward pivotFeedForward = new ArmFeedforward(0, 0.027576445, 0.001, 0);
  //RotationalFeedForward pivotFeedForward = new RotationalFeedForward(0, 1, 0.001, 0.027576445);
  double LineOfBestFitCalculation;
  NeutralModeValue Coast = NeutralModeValue.Coast;
  double AmpShooterRPM;

  /** Creates a new ShooterSubsystem. */

  public ShooterSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 0.7;//.7;
    configs.Slot0.kI = 1.5;
    configs.Slot0.kD = 0;

    configs.Slot1.kP = .35;
    configs.Slot1.kI = 1;
    configs.Slot1.kD = 0;

    configs.Voltage.PeakForwardVoltage = 10;
    configs.Voltage.PeakReverseVoltage = -10;

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    configs.MotorOutput.NeutralMode = Coast;
    shooterMotor.getConfigurator().apply(configs);
    shooterPivotMotorMaster.setInverted(true);
    shooterPivotMotorFollower.setControl(new Follower(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN, true));
    shooterPivotMotorMaster.setPosition(0);
    shooterPivotMotorFollower.setPosition(0);
  }

  public void ShootAtSubwoofer() {
    shooterPivotMotorMaster
        .set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shooterAngleSubwooferConstant)
            + pivotFeedForward.calculate(Constants.Shooter.shooterAngleSubwooferConstant * 6.2832, 2));
    shooterMotor.setControl(shooterVelocityFast.withVelocity(-4000 / 60));
  }

  public void ShootAtFarshot() {
    shooterPivotMotorMaster 
        .set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shooterAngleFarshotConstant)
            + pivotFeedForward.calculate(Constants.Shooter.shooterAngleFarshotConstant * 6.2832, 2));
    shooterMotor.setControl(shooterVelocityFast.withVelocity(-4000 / 60));
  }

  public void ShootWithLimelight() {
    LineOfBestFitCalculation = (((Math.tan((Math.toRadians(LimelightHelpers.getTY("") + 29)) / 45.5)) - 0.01795)
        / -0.054);
    shooterMotor.setControl(shooterVelocityFast.withVelocity(-3750 / 60));
    shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle, LineOfBestFitCalculation)
        + pivotFeedForward.calculate(LineOfBestFitCalculation * 6.2832, 2));
  }

  public void ShooterShoot() {
    shooterMotor.setControl(shooterVelocityFast.withVelocity(-4000 / 60));
  }

  public void StopShooter() {
    shooterMotor.set(0);
  }

  public boolean ReadyToShoot() {
    return (MathUtil.isNear(LineOfBestFitCalculation, shooterMotorAngle, .005)
        && MathUtil.isNear(-3750 / 60, shooterMotor.getVelocity().getValueAsDouble(), 1));
  }

  public void ShooterPrepareToIndex() {
    shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle,
        Constants.Shooter.targetShooterPivotIndexAngle)
        + pivotFeedForward.calculate(Constants.Shooter.targetShooterPivotIndexAngle * 6.2832, 2));
  }

  public void ShootInAmp() {
    shooterPivotMotorMaster.set( -shooterPivotPID.calculate(shooterMotorAngle, .45)
        + pivotFeedForward.calculate(.45* 6.2832, 2));
  //  shooterMotor.setControl(shooterVelocitySLow.withVelocity(-AmpShooterRPM / 60));
  }

  public void ShooterDown() {
    shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle, .05)
        + pivotFeedForward.calculate(.05 * 6.2832, 2));
  }

  public void ShootOverStage() {
    shooterPivotMotorMaster
        .set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shootOverStageAngleConstant)
            + pivotFeedForward.calculate(Constants.Shooter.shootOverStageAngleConstant * 6.2832, 2));
    shooterMotor.setControl(shooterVelocitySLow.withVelocity(-3300 / 60));
  }

  public void ShootTrapdoor() {
    shooterPivotMotorMaster
        .set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shootTrapdoorAngleConstant)
            + pivotFeedForward.calculate(Constants.Shooter.shootTrapdoorAngleConstant * 6.2832,2));
    shooterMotor.setControl(shooterVelocitySLow.withVelocity(-3200 / 60)); //3300/60 works for close shot without limelight
  }

   

  @Override
  public void periodic() {
    SmartDashboard.getNumber("kP RPM", 0);
    SmartDashboard.getNumber("kI RPM", 0);
    SmartDashboard.getNumber("kD RPM", 0);

    AmpShooterRPM = SmartDashboard.getNumber("AmpShooterRpm", 1700);
    LineOfBestFitCalculation = (((Math.tan((Math.toRadians(LimelightHelpers.getTY("") + 29)) / 45.5)) - 0.019)
        / -0.0566);

    shooterMotorAngle = shooterCANcoder.getAbsolutePosition().getValueAsDouble();
    shooterCurrentRPM = (shooterMotor.getVelocity().getValueAsDouble() * 60);

    if ((shooterMotorAngle > Constants.Shooter.targetShooterPivotIndexAngle - 0.01)
        && (shooterMotorAngle < Constants.Shooter.targetShooterPivotIndexAngle + 0.01)) {
      shooterReadyToIndex = true;
    } else {
      shooterReadyToIndex = false;
    }
    SmartDashboard.putNumber("line of best fit calculation", LineOfBestFitCalculation);
    SmartDashboard.putNumber("shooter angle with CANcoder", shooterMotorAngle);
    SmartDashboard.putNumber("current shooter RPM", shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("distance with limelight",
        Math.tan((Math.toRadians(LimelightHelpers.getTY("") + 29)) / 45.5));
    // This method will be called once per scheduler run
  }
}
