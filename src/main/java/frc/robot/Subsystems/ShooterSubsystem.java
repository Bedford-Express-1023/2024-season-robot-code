// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.LinearInterpolator;
import frc.robot.LinearInterpolator;
import frc.robot.RotationalFeedForward;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(Constants.Shooter.SHOOTER_MOTOR_CAN);
  private final TalonFX shooterPivotMotorMaster = new TalonFX(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN); //left pivot motor
  private final TalonFX shooterPivotMotorFollower = new TalonFX(Constants.Shooter.SHOOTER_RIGHT_PIVOT_CAN); //right pivot motor
  private final CANcoder shooterCANcoder = new CANcoder(Constants.Shooter.SHOOTER_CANCODER_ID);

  //public static final LinearInterpolator shooterInterpolator = new LinearInterpolator(Constants.Shooter.shooterTable);
  public static final LinearInterpolator shooterPivotInterpolator = new LinearInterpolator(Constants.Shooter.pivotTable);
  double kp;
double ki;
double kd;
private double shooterMotorAngle;
  public double shooterCurrentAngle; //in degrees
  private double shooterTargetAngle; //in degrees
  public double shooterCurrentRPM;
  private double shooterTargetRPM;
  private double limelightTY;
  public boolean shooterReadyToIndex;
  public Slot0Configs slot0Configs = new Slot0Configs();
  public Slot1Configs slot1Configs = new Slot1Configs();
  public VelocityVoltage shooterVelocity = new VelocityVoltage(Constants.Shooter.shooterVelocitySubwooferConstant, 0, false, 0, 0, false, false, false);
 public PIDController shooterPivotPID = new PIDController(.85,0.075,0.0001);
  public final RotationalFeedForward feedForward = new RotationalFeedForward(0,1,0, -0.3);

 NeutralModeValue brake = NeutralModeValue.Brake;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
   // slot0Configs.kV = 1; //shooter config velocity feedforward gain
    // slot0Configs.kP = 2; //shooter config proportional gain
    // slot0Configs.kI = 0.25; //shooter config integral gain
    // slot0Configs.kD = 0.001; //shooter config derivative gain
    // slot0Configs.kP = 2; //shooter config proportional gain
    //slot0Configs.kI = .5; //shooter config integral gain
   // slot0Configs.kD = kd; //shooter config derivative gain
 TalonFXConfiguration configs = new TalonFXConfiguration();
   configs.Slot0.kP = .7;
   configs.Slot0.kI = 2;
   configs.Slot0.kD = 0;

    configs.Voltage.PeakForwardVoltage = 10;
    configs.Voltage.PeakReverseVoltage = -10;
   
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    configs.MotorOutput.NeutralMode = brake;
   shooterMotor.getConfigurator().apply(configs);
   shooterPivotMotorMaster.setInverted(true);
   
  //  slot1Configs.kV = 1; //pivot config velocity feedforward gain
    // slot1Configs.kP = 0.1; //pivot config proportional gain
    // slot1Configs.kI = 0.05; //pivot config integral gain
    // slot1Configs.kD = 0.01; //pivot config derivative gain

    // shooterMotor.getConfigurator().apply(slot0Configs, 0.050); 
    // shooterPivotMotorMaster.getConfigurator().apply(slot1Configs, 0.050); 
    // shooterPivotMotorFollower.getConfigurator().apply(slot1Configs, 0.050); 
   shooterPivotMotorFollower.setControl(new Follower(Constants.Shooter.SHOOTER_LEFT_PIVOT_CAN, true)); //config right pivot motor follows left pivot motor
   shooterPivotMotorMaster.setPosition(0);
   shooterPivotMotorFollower.setPosition(0);
  }

// the max is 4350 rpm dont forget

  public void ShootAtSubwoofer() {
//Constants.Shooter.shooterVelocitySubwooferConstant)); //Subwoofer RPM is 4000
      shooterPivotMotorMaster.set(shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shooterAngleSubwooferConstant));
      SmartDashboard.putNumber("intake up power",shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shooterAngleSubwooferConstant));
     // shooterPivotMotorMaster.set(shooterPivotPID.calculate(shooterMotorAngle, -0.06));
  }

  public void ShootAtPlatform() {
      shooterMotor.setControl(shooterVelocity.withVelocity(Constants.Shooter.shooterVelocityPlatformConstant)); 
  }

  public void ShootWithLimelight() {
            shooterMotor.setControl(shooterVelocity.withVelocity(-4000/60));
            shooterPivotMotorMaster.set(shooterPivotPID.calculate(shooterMotorAngle, shooterTargetAngle));
  }

 public void ShooterShoot() {
        shooterMotor.setControl(shooterVelocity.withVelocity(-4000/60));
         SmartDashboard.putNumber("shooter velocity RPS", shooterMotor.getVelocity().getValueAsDouble());
  }

  public void StopShooter() {
       shooterMotor.set(0);
  }

  public void ShooterPrepareToIndex() {
      shooterPivotMotorMaster.set(shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.targetShooterPivotIndexAngle));
      SmartDashboard.putNumber("shooter to index power", shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.targetShooterPivotIndexAngle));
  }

  public void PointTowardsSpeaker() {
    shooterPivotMotorMaster.set(shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.targetShooterPivotIndexAngle));
     SmartDashboard.putNumber("shooter angle pid", shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.targetShooterPivotIndexAngle));
  }

public void ShootInAmp() {
     shooterMotor.setControl(shooterVelocity.withVelocity(-Constants.Shooter.ampRPM/60));
    }

  @Override
  public void periodic() {
    shooterPivotPID.setPID(kp,ki,kd);

      kp = SmartDashboard.getNumber("shooter kp",0);
      ki = SmartDashboard.getNumber("shooter ki",0);
      kd = SmartDashboard.getNumber("shooter kd",0);

   shooterMotorAngle = shooterCANcoder.getAbsolutePosition().getValueAsDouble();
    shooterCurrentRPM = (shooterMotor.getVelocity().getValueAsDouble() * 60);
    

     if ((shooterMotorAngle > Constants.Shooter.targetShooterPivotIndexAngle - 0.01) && (shooterMotorAngle < Constants.Shooter.targetShooterPivotIndexAngle + 0.01)) {
       shooterReadyToIndex = true;
     }
     else {
       shooterReadyToIndex = false;
    }

    limelightTY = LimelightHelpers.getTY("");
    shooterTargetAngle = shooterPivotInterpolator.getInterpolatedValue(LimelightHelpers.getTY("")); //FIXME insert current limelight y axis data

    //shooterTargetRPM = shooterInterpolator.getInterpolatedValue(0); //FIXME insert current limelight y axis data
    //shooterTargetAngle = shooterPivotInterpolator.getInterpolatedValue(0); //FIXME insert current limelight y axis data

    SmartDashboard.putNumber("shooter angle with CANcoder",shooterMotorAngle);
  //  SmartDashboard.putNumber("shooter angle pid", shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.targetShooterPivotIndexAngle));
    SmartDashboard.putNumber("current shooter RPM", shooterCurrentRPM);
    SmartDashboard.putNumber("target shooter RPM", shooterTargetRPM);
    SmartDashboard.putNumber("target shooter angle", shooterTargetAngle);

    SmartDashboard.putNumber("left pivot motor position", shooterPivotMotorMaster.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("right pivot motor position", shooterPivotMotorFollower.getPosition().getValueAsDouble());

   // SmartDashboard.putNumber("pivot current velocity", shooterPivotPID.calculate(shooterMotorAngle, -0.055));

    SmartDashboard.putNumber("interpolated value", shooterTargetAngle);
    SmartDashboard.putNumber("Limelight TY value", LimelightHelpers.getTY(""));
    // This method will be called once per scheduler run
  }
}
