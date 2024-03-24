// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  // private final TalonFXConfiguration climberMotorConfig = new
  // TalonFXConfiguration();
  // private final CurrentLimitsConfigs climberCurLimConfig =
  // climberMotorConfig.CurrentLimits;

  private final TalonFX rightClimberMotor = new TalonFX(Constants.Climber.CLIMBER_RIGHT_CAN); // FIXME
  private final TalonFX leftClimberMotor = new TalonFX(Constants.Climber.CLIMBER_LEFT_CAN); // FIXME
  DigitalInput rightLimitSwitch = new DigitalInput(5);
  DigitalInput leftLimitSwitch = new DigitalInput(6);
NeutralModeValue Brake = NeutralModeValue.Brake;
  //private StatusSignal<Double> rightClimberMotorStatorCurrent;
  //private StatusSignal<Double> leftClimberMotorStatorCurrent;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
     TalonFXConfiguration configs = new TalonFXConfiguration();
     configs.MotorOutput.NeutralMode = Brake;
     configs.CurrentLimits.StatorCurrentLimitEnable = true;
     configs.CurrentLimits.StatorCurrentLimit = 15
     ;// 30 StatorCurrentLimit for the climber in the box
     configs.CurrentLimits.SupplyCurrentLimitEnable = true;
     configs.CurrentLimits.SupplyCurrentLimit = 3; //6  for the climber in the box
     rightClimberMotor.getConfigurator().apply(configs);
     leftClimberMotor.getConfigurator().apply(configs);
    /*
     * climberCurLimConfig.StatorCurrentLimit =
     * Constants.Climber.climberStatorCurrentLimit;
     * climberCurLimConfig.StatorCurrentLimitEnable = false;
     * climberMotorConfig.withCurrentLimits(climberCurLimConfig);
     * rightClimberMotor.getConfigurator().apply(climberCurLimConfig);
     * leftClimberMotor.getConfigurator().apply(climberCurLimConfig);
     */
  }

  public void ClimberUp() {
    rightClimberMotor.set(0.10);// 50% for the climber in a box
    leftClimberMotor.set(-0.10);
  }

  public void ClimberDown() {
    rightClimberMotor.set(-0.20);// 85% for the climber in the box
    leftClimberMotor.set(0.20);
  }

  public void ClimberDownWithSwitch() {
 if(leftLimitSwitch.get() == false){
leftClimberMotor.set(.2);
 }
 else if (leftLimitSwitch.get() == true){
  leftClimberMotor.set(0);
 }
 if(rightLimitSwitch.get() == false){
  rightClimberMotor.set(-.2);
 }
 else if (rightLimitSwitch.get() == true){
rightClimberMotor.set(0);
 }
  }

    public void ClimberUpWithSwitch() {
 if(leftLimitSwitch.get() == false){
leftClimberMotor.set(-.1);
 }
 else if (leftLimitSwitch.get() == true){
  leftClimberMotor.set(0);
 }
 if(rightLimitSwitch.get() == false){
  rightClimberMotor.set(.1);
 }
 else if (rightLimitSwitch.get() == true){
rightClimberMotor.set(0);
 }
  }
  public void ClimberStop(){
    leftClimberMotor.set(0);
    rightClimberMotor.set(0);
  }

  @Override
  public void periodic() {
//    rightClimberMotorStatorCurrent = rightClimberMotor.getStatorCurrent();
//    leftClimberMotorStatorCurrent = leftClimberMotor.getStatorCurrent();

    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Left Climer Motor Limit Switch", leftLimitSwitch.get());
     SmartDashboard.putBoolean("Right Climer Motor Limit Switch", rightLimitSwitch.get());
  }
}
