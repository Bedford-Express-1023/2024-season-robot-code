// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  public final TalonFX shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_CAN);
  public final TalonFX shooterPivotMotorMaster = new TalonFX(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN);
  public final TalonFX shooterPivotMotorFollower = new TalonFX(Constants.Shooter.SHOOTER_RIGHT_PIVOT_CAN);
  public final CANcoder shooterCANcoder = new CANcoder(Constants.Shooter.SHOOTER_CANCODER_ID);
  private double rotationTolerance;
  private double shooterMotorAngle;
  public double shooterCurrentAngle; // in degrees
  public double shooterCurrentRPM;
  public double limelightTX;
  public boolean shooterReadyToIndex;
  public Slot0Configs slot0Configs = new Slot0Configs();
  public Slot1Configs slot1Configs = new Slot1Configs();
  public VelocityVoltage shooterVelocityFast = new VelocityVoltage(Constants.Shooter.shooterVelocitySubwooferConstant,
      0, false, 0, 0, false, false, false);
  public VelocityVoltage shooterVelocitySLow = new VelocityVoltage(Constants.Shooter.shooterVelocitySubwooferConstant,
      0, false, 0, 1, false, false, false);
  public PIDController shooterPivotPID = new PIDController(4.2, 0.75, 0);// (.85,0.075,0.0001);
  public PIDController AmpPivotPID = new PIDController(2, 0.1, 0);
  ArmFeedforward pivotFeedForward = new ArmFeedforward(0, -0.02636717, 0, 0); // 0.027576445
  double LineOfBestFitCalculation;
  NeutralModeValue Coast = NeutralModeValue.Coast;
  NeutralModeValue Brake = NeutralModeValue.Brake;
  InvertedValue Invert = InvertedValue.Clockwise_Positive;
  double AmpShooterRPM;
  double shooterPivotTolerance;
  double FeederRPMLine;
double feederRotationLine;
  public ShooterSubsystem() {
    TalonFXConfiguration Shooterconfigs = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    Shooterconfigs.Slot0.kP = 0.7;
    Shooterconfigs.Slot0.kI = 1.8;
    Shooterconfigs.Slot0.kD = 0;

    Shooterconfigs.Slot1.kP = .35;
    Shooterconfigs.Slot1.kI = .9;
    Shooterconfigs.Slot1.kD = 0;

    Shooterconfigs.MotorOutput.NeutralMode = Coast;

    pivotConfigs.MotorOutput.PeakForwardDutyCycle = .3;
    pivotConfigs.MotorOutput.PeakReverseDutyCycle = -.3;
    pivotConfigs.MotorOutput.NeutralMode = Brake;
    pivotConfigs.MotorOutput.Inverted = Invert;

    shooterMotor.getConfigurator().apply(Shooterconfigs);
    shooterPivotMotorMaster.getConfigurator().apply(pivotConfigs);
    shooterPivotMotorFollower.getConfigurator().apply(pivotConfigs);

    shooterPivotMotorFollower.setControl(new Follower(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN, true));

  }

  public void shooterZero() {
    shooterPivotMotorMaster
        .set(-shooterPivotPID.calculate(shooterMotorAngle, 0)
            + pivotFeedForward.calculate(0 * 6.2832, 1));
  }

  public void ShootAtSubwoofer() {
    shooterPivotMotorMaster
        .set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shooterAngleSubwooferConstant)
            + pivotFeedForward.calculate(Constants.Shooter.shooterAngleSubwooferConstant * 6.2832, 2));
    shooterMotor.setControl(shooterVelocityFast.withVelocity(-4000 / 60));
  }
  public boolean ReadyToShootAtSubwoofer(){
    if (MathUtil.isNear(Constants.Shooter.shooterAngleSubwooferConstant, shooterMotorAngle, .01)&& 
    MathUtil.isNear(-4000/60, shooterCurrentRPM, 2)){
      return true;
    }
    else{
      return  false;
    }
  }

  public void ShootAtFarshot() {
    shooterPivotMotorMaster
        .set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shooterAngleFarshotConstant)
            + pivotFeedForward.calculate(Constants.Shooter.shooterAngleFarshotConstant * 6.2832, 2));
    shooterMotor.setControl(shooterVelocityFast.withVelocity(-4000 / 60));
  }

  public void ShootWithLimelight() {
    shooterMotor.setControl(shooterVelocityFast.withVelocity(-4500 / 60));// 3750
    shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle, LineOfBestFitCalculation)
        + pivotFeedForward.calculate(LineOfBestFitCalculation * 6.2832, 1));
  }

  public void ShooterShoot() {
    shooterMotor.setControl(shooterVelocitySLow.withVelocity(-3750/ 60));
        shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle,
        Constants.Shooter.targetShooterPivotIndexAngle)
        + pivotFeedForward.calculate(Constants.Shooter.targetShooterPivotIndexAngle * 6.2832, 1));
  }

  public void StopShooter() {
    shooterMotor.set(0);
        shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle,
        Constants.Shooter.targetShooterPivotIndexAngle)
        + pivotFeedForward.calculate(Constants.Shooter.targetShooterPivotIndexAngle * 6.2832, 1));
  }
public void StopShooterPivot(){
  shooterPivotMotorMaster.set(0);
}  
public boolean ReadyToShoot() {
    if (MathUtil.isNear(LineOfBestFitCalculation, shooterMotorAngle, shooterPivotTolerance)
        && MathUtil.isNear(-4500 / 60, shooterMotor.getVelocity().getValueAsDouble(), 1)// 3750
     && MathUtil.isNear(0, limelightTX, 3)
    ) {
      return true;
    } else {
      return false;
    }

  }

  public boolean ReadyToShootAutofaster() {
    if (MathUtil.isNear(LineOfBestFitCalculation, shooterMotorAngle, .02)// .005
        && MathUtil.isNear(-4500 / 60, shooterMotor.getVelocity().getValueAsDouble(), 4))// 1
    {
      return true;
    } else {
      return false;
    }

  }

  public boolean ReadyToShootAutoSlower() {
    if (MathUtil.isNear(LineOfBestFitCalculation, shooterMotorAngle, .004)// .005
        && MathUtil.isNear(-4500 / 60, shooterMotor.getVelocity().getValueAsDouble(), 1))// 1
    {
      return true;
    } else {
      return false;
    }

  }

  public void ShootUnderStage() {
    shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle,
        0)
        + pivotFeedForward.calculate(0 * 6.2832, 1));
    shooterMotor.setControl(shooterVelocitySLow.withVelocity(-4500 / 60));
  }

  public boolean ReadyToShootUnderStage() {
    return MathUtil.isNear(-4500 / 60, shooterMotor.getVelocity().getValueAsDouble(), 6)
        && (MathUtil.isNear(0, shooterMotorAngle, .03));

  }

  public boolean AprilTagSeen() {
    return limelightTX > 0;
  }

  public void ShooterPrepareToIndex() {
    shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle,
        Constants.Shooter.targetShooterPivotIndexAngle)
        + pivotFeedForward.calculate(Constants.Shooter.targetShooterPivotIndexAngle * 6.2832, 1));
  }

  public void ShootInAmp() {
    shooterPivotMotorMaster.set(-AmpPivotPID.calculate(shooterMotorAngle, .27)// .319 for trap
        + pivotFeedForward.calculate(.27 * 6.2832, 1));
  }

  public void ShooterDown() {
    shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle, -.155)
        + pivotFeedForward.calculate(-.155 * 6.2832, 1));
  }

  public void ShootOverStage() {
    shooterPivotMotorMaster
        .set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shootOverStageAngleConstant)
            + pivotFeedForward.calculate(Constants.Shooter.shootOverStageAngleConstant * 6.2832, 1));
    shooterMotor.setControl(shooterVelocitySLow.withVelocity(-3500 / 60));
  }
  public boolean ReadyToShootOverStage() {
    if (MathUtil.isNear(Constants.Shooter.shootOverStageAngleConstant, shooterMotorAngle, .02)
        && MathUtil.isNear(FeederRPMLine / 60, shooterMotor.getVelocity().getValueAsDouble(), 1)// 3750
     && MathUtil.isNear(feederRotationLine, limelightTX, 3)
    ) {
      return true;
    } else {
      return false;
    }

  }
  public boolean ReadyToShootOverStageAuto(){
    if(MathUtil.isNear(-.1, shooterMotorAngle, .05) 
    && MathUtil.isNear(-4000/60,  shooterMotor.getVelocity().getValueAsDouble(), 3)){
      return true;
    }
   else {
      return false;
    }
  }

  public void ShootTrapdoor() {
    shooterPivotMotorMaster
        .set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shootTrapdoorAngleConstant)
            + pivotFeedForward.calculate(Constants.Shooter.shootTrapdoorAngleConstant * 6.2832, 1));
    shooterMotor.setControl(shooterVelocityFast.withVelocity(-4250 / 60)); // 3300/60 works for close shot without
                                                                           // limelight
  }

  public void ShooterToFirstClimb(){
     shooterPivotMotorMaster.set(-AmpPivotPID.calculate(shooterMotorAngle, .08)// .319 for trap
        + pivotFeedForward.calculate(.08 * 6.2832, 1));
  }
 public void ShooterToSecondClimb(){
     shooterPivotMotorMaster.set(-AmpPivotPID.calculate(shooterMotorAngle, .330)// .319 for trap
        + pivotFeedForward.calculate(.330 * 6.2832, 1));
  }

  @Override
  public void periodic() {
    double distanceWithLimelight =  Math.tan((Math.toRadians(LimelightHelpers.getTY("limelight-shooter") + 29)) / 45.5);
    //rotationTolerance = (Math.tan((Math.toRadians(LimelightHelpers.getTY("") + 29)) / 45.5)) * 550- 1.39869;
    limelightTX = LimelightHelpers.getTX("limelight-shooter");
FeederRPMLine = 260000 * distanceWithLimelight -4785.31;

    AmpShooterRPM = SmartDashboard.getNumber("AmpShooterRpm", 1700);
    LineOfBestFitCalculation = (((Math.tan((Math.toRadians(LimelightHelpers.getTY("limelight-shooter") + 29)) / 45.5)) + 0.0035)// .0048
        / -0.113067);
    // -.1325
    shooterMotorAngle = shooterCANcoder.getAbsolutePosition().getValueAsDouble();
    shooterCurrentRPM = (shooterMotor.getVelocity().getValueAsDouble() * 60);

    if ((shooterMotorAngle > Constants.Shooter.targetShooterPivotIndexAngle - 0.01)
        && (shooterMotorAngle < Constants.Shooter.targetShooterPivotIndexAngle + 0.01)) {
      shooterReadyToIndex = true;
    } else {
      shooterReadyToIndex = false;

    }

if(distanceWithLimelight <.007){
shooterPivotTolerance = .006;
}
else if (distanceWithLimelight <.006){
  shooterPivotTolerance = .006;
}
else{
  shooterPivotTolerance = .006;
}
if (distanceWithLimelight > .0119){
  rotationTolerance = 5;
}
else{
rotationTolerance = 1.5;
}

feederRotationLine = -4000 * distanceWithLimelight +18; 


                  
    SmartDashboard.putNumber("line of best fit calculation", LineOfBestFitCalculation);
    SmartDashboard.putNumber("shooter angle with CANcoder", shooterMotorAngle);
    SmartDashboard.putNumber("current shooter RPM", shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("distance with limelight", distanceWithLimelight);
    SmartDashboard.putNumber("FeederRpm",FeederRPMLine);
    SmartDashboard.putNumber("LimlightTX", limelightTX);
    SmartDashboard.putNumber("rotation tolerance", rotationTolerance);
    // This method will be called once per scheduler run
  }
  
}