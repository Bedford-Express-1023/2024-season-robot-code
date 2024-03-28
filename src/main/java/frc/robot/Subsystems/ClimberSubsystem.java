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

  private final TalonFX rightClimberMotor = new TalonFX(Constants.Climber.CLIMBER_RIGHT_CAN); // FIXME
  private final TalonFX leftClimberMotor = new TalonFX(Constants.Climber.CLIMBER_LEFT_CAN); // FIXME
  DigitalInput rightLimitSwitch = new DigitalInput(4);
  DigitalInput leftLimitSwitch = new DigitalInput(5);
NeutralModeValue Brake = NeutralModeValue.Brake;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
     TalonFXConfiguration configs = new TalonFXConfiguration();
     configs.MotorOutput.NeutralMode = Brake;
     configs.CurrentLimits.StatorCurrentLimitEnable = true;
     configs.CurrentLimits.StatorCurrentLimit = 30;// 30 StatorCurrentLimit for the climber in the box
     configs.CurrentLimits.SupplyCurrentLimitEnable = true;
     configs.CurrentLimits.SupplyCurrentLimit = 6; //6  for the climber in the box
     rightClimberMotor.getConfigurator().apply(configs);
     leftClimberMotor.getConfigurator().apply(configs);

  }

  public void ClimberUp() {
    rightClimberMotor.set(0.850);// 50% for the climber in a box
    leftClimberMotor.set(-0.850);
  }

  public void ClimberDown() {
    rightClimberMotor.set(-0.20);// 85% for the climber in the box
    leftClimberMotor.set(0.20);
  }

  public void ClimberDownWithSwitch() {
 if(leftLimitSwitch.get() == true){
leftClimberMotor.set(.85);
 }
 else if (leftLimitSwitch.get() == false){
  leftClimberMotor.set(0);
 }
 if(rightLimitSwitch.get() == true){
  rightClimberMotor.set(-.85);
 }
 else if (rightLimitSwitch.get() == false){
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
    SmartDashboard.putBoolean("Left Climer Motor Limit Switch", leftLimitSwitch.get());
     SmartDashboard.putBoolean("Right Climer Motor Limit Switch", rightLimitSwitch.get());
  }
}
