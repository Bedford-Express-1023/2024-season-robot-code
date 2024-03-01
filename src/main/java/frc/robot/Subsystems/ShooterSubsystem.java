// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.NavigableMap;
import java.util.TreeMap;

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

import edu.wpi.first.math.controller.ArmFeedforward;
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
  public double limelightTY;
  public boolean shooterReadyToIndex;
  private double targetShooterPivotAngle;
  public Slot0Configs slot0Configs = new Slot0Configs();
  public Slot1Configs slot1Configs = new Slot1Configs();
  public VelocityVoltage shooterVelocityFast = new VelocityVoltage(Constants.Shooter.shooterVelocitySubwooferConstant, 0, false, 0, 0, false, false, false);
  public VelocityVoltage shooterVelocitySLow = new VelocityVoltage(Constants.Shooter.shooterVelocitySubwooferConstant, 0, false, 0, 1, false, false, false);
 public PIDController shooterPivotPID = new PIDController(1.7,0.28,0);//(.85,0.075,0.0001);
  ArmFeedforward pivotFeedForward = new ArmFeedforward(0, 0.027576445, 0.001, 0);
 double LineOfBestFitCalculation;
 NeutralModeValue brake = NeutralModeValue.Brake;

 
  private NavigableMap<Double, Double> shooterPivotAngles = new TreeMap<Double, Double>();

 
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

      configs.Slot1.kP = .35;
   configs.Slot1.kI = 1;
   configs.Slot1.kD = 0;

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
   setUpLookUpTable();
  }

// the max is 4350 rpm dont forget

  private void setUpLookUpTable() {
    shooterPivotAngles.put(1.4, 0.106);
    shooterPivotAngles.put(1.53, 0.03);
    shooterPivotAngles.put(1.61, -0.012);
    shooterPivotAngles.put(1.78, -0.032);
    shooterPivotAngles.put(2.06, -0.048);
    shooterPivotAngles.put(2.1, -0.055);
  }

  public void ShootAtSubwoofer() {
//Constants.Shooter.shooterVelocitySubwooferConstant)); //Subwoofer RPM is 4000
      shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shooterAngleSubwooferConstant) 
                                + pivotFeedForward.calculate(Constants.Shooter.shooterAngleSubwooferConstant *  6.2832, 2));
      shooterMotor.setControl(shooterVelocityFast.withVelocity(-4000/60));
      SmartDashboard.putNumber("intake up power",shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shooterAngleSubwooferConstant));
     // shooterPivotMotorMaster.set(shooterPivotPID.calculate(shooterMotorAngle, -0.06));
  }

  public void ShootAtPlatform() {
      //   shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle,) 
      //                           + pivotFeedForward.calculate(-.1 *  6.2832, 2)); 
      //  shooterMotor.setControl(shooterVelocity.withVelocity(-4000/60));
  }

    public void ShootAtFarshot() {
//Constants.Shooter.shooterVelocitySubwooferConstant)); //Subwoofer RPM is 4000
      shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.shooterAngleFarshotConstant) 
                                +pivotFeedForward.calculate(Constants.Shooter.shooterAngleFarshotConstant *  6.2832, 2));
      shooterMotor.setControl(shooterVelocityFast.withVelocity(-4000/60));
  }

  public void ShootWithLimelight() {
    LineOfBestFitCalculation = (((Math.tan((Math.toRadians(LimelightHelpers.getTY("") + 29))/45.5))-0.019)/-0.0566);
    shooterMotor.setControl(shooterVelocityFast.withVelocity(-4000/60));
    shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle, LineOfBestFitCalculation)
                              + pivotFeedForward.calculate(LineOfBestFitCalculation *  6.2832, 2));

  }

 public void ShooterShoot() {
        shooterMotor.setControl(shooterVelocityFast.withVelocity(-4000/60));
         SmartDashboard.putNumber("shooter velocity RPS", shooterMotor.getVelocity().getValueAsDouble());
  }

  public void StopShooter() {
       shooterMotor.set(0);
  }

  public void ShooterPrepareToIndex() {
    double pidCalculationOutPut = shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.targetShooterPivotIndexAngle);
    double FeedShooterCalculationOutPut = pivotFeedForward.calculate(Constants.Shooter.targetShooterPivotIndexAngle *  6.2832, 2);
      shooterPivotMotorMaster.set(-pidCalculationOutPut 
                                + FeedShooterCalculationOutPut);
      SmartDashboard.putNumber("shooter to index power", shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.targetShooterPivotIndexAngle));
      SmartDashboard.putNumber("PID calculation Output", pidCalculationOutPut);
      SmartDashboard.putNumber("Feed forward calculation output", FeedShooterCalculationOutPut);
      SmartDashboard.putNumber("Final shooter calculation", pidCalculationOutPut - FeedShooterCalculationOutPut);
  }

  public void PointTowardsSpeaker() {/* 
    shooterPivotMotorMaster.set(shooterPivotPID.calculate(shooterMotorAngle, getShooterPivotAngle()) 
                                - pivotFeedForward.calculate(getShooterPivotAngle() *  6.2832, 2));
     SmartDashboard.putNumber("shooter angle pid", targetShooterPivotAngle); */
  }

public void ShootInAmp() {
       shooterPivotMotorMaster.set(-shooterPivotPID.calculate(shooterMotorAngle, .005) 
                                + pivotFeedForward.calculate(.005 *  6.2832, 2));
     shooterMotor.setControl(shooterVelocitySLow.withVelocity(-2100/60));
    }
/* 
  private double getShooterPivotAngle() {
    double closeDistance = shooterPivotAngles.floorKey(limelightTY);
    double farDistance = shooterPivotAngles.ceilingKey(limelightTY);
  
    double closeAngle = shooterPivotAngles.floorEntry(limelightTY).getValue();
    double farAngle = shooterPivotAngles.ceilingEntry(limelightTY).getValue();
    return ((farAngle - closeAngle) / (farDistance - closeDistance))
        * (limelightTY - farDistance) + farAngle;
  }
*/
  @Override
  public void periodic() {
    //targetShooterPivotAngle = getShooterPivotAngle();
   
 LineOfBestFitCalculation = (((Math.tan((Math.toRadians(LimelightHelpers.getTY("") + 29))/45.5))-0.019)/-0.0566);
 SmartDashboard.putNumber("Line of best fit calculation" ,LineOfBestFitCalculation) ;
   

  

   shooterMotorAngle = shooterCANcoder.getAbsolutePosition().getValueAsDouble();
    shooterCurrentRPM = (shooterMotor.getVelocity().getValueAsDouble() * 60);
    

     if ((shooterMotorAngle > Constants.Shooter.targetShooterPivotIndexAngle - 0.01) && (shooterMotorAngle < Constants.Shooter.targetShooterPivotIndexAngle + 0.01)) {
       shooterReadyToIndex = true;
     }
     else {
       shooterReadyToIndex = false;
    }

    limelightTY = LimelightHelpers.getTY("");
    shooterTargetAngle = shooterPivotInterpolator.getInterpolatedValue(Math.tan((Math.toRadians(LimelightHelpers.getTY("") + 29))/45.5)); //FIXME insert current limelight y axis data

    //shooterTargetRPM = shooterInterpolator.getInterpolatedValue(0); //FIXME insert current limelight y axis data
    //shooterTargetAngle = shooterPivotInterpolator.getInterpolatedValue(0); //FIXME insert current limelight y axis data

    SmartDashboard.putNumber("shooter angle with CANcoder",shooterMotorAngle);
  //  SmartDashboard.putNumber("shooter angle pid", shooterPivotPID.calculate(shooterMotorAngle, Constants.Shooter.targetShooterPivotIndexAngle));
    SmartDashboard.putNumber("current shooter RPM", shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("target shooter RPM", shooterTargetRPM);

    SmartDashboard.putNumber("left pivot motor position", shooterPivotMotorMaster.getPosition().getValueAsDouble());

   // SmartDashboard.putNumber("pivot current velocity", shooterPivotPID.calculate(shooterMotorAngle, -0.055));

    SmartDashboard.putNumber("interpolated value", shooterTargetAngle);
    SmartDashboard.putNumber("distance with limelight", Math.tan((Math.toRadians(LimelightHelpers.getTY("") + 29))/45.5));
    SmartDashboard.putNumber("Limelight TY value", LimelightHelpers.getTY(""));
    // This method will be called once per scheduler run
  }
}
