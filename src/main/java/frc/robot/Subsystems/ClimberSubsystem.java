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
  DigitalInput rightLimitSwitch = new DigitalInput(7);//4 for the box climber
  DigitalInput leftLimitSwitch = new DigitalInput(3);// 5 for the box climber
NeutralModeValue Brake = NeutralModeValue.Brake;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
     TalonFXConfiguration configs = new TalonFXConfiguration();
     configs.MotorOutput.NeutralMode = Brake;
     configs.CurrentLimits.StatorCurrentLimitEnable = true;
     configs.CurrentLimits.StatorCurrentLimit = 25;// 30 StatorCurrentLimit for the climber in the box
     configs.CurrentLimits.SupplyCurrentLimitEnable = true;
     configs.CurrentLimits.SupplyCurrentLimit = 8.5; //6  for the climber in the box
     rightClimberMotor.getConfigurator().apply(configs);
     leftClimberMotor.getConfigurator().apply(configs);

  }

  public void ClimberUp() {
    rightClimberMotor.set(0.10);// 850% for the climber in a box
    leftClimberMotor.set(-0.10);
  }

  public void ClimberDown() {
    rightClimberMotor.set(-0.10);// 85% for the climber in the box
    leftClimberMotor.set(0.10);
  }

  public void ClimberDownWithSwitch() {
 if(leftLimitSwitch.get() == true){
leftClimberMotor.set(.8);
 }
 else if (leftLimitSwitch.get() == false){
  leftClimberMotor.set(0);
 }
 if(rightLimitSwitch.get() == true){
  rightClimberMotor.set(-.8);
 }
 else if (rightLimitSwitch.get() == false){
rightClimberMotor.set(0);
 }
}
    public void ClimberUpWithSwitch() {
    if (rightClimberMotor.getPosition().getValueAsDouble() < 185){
      rightClimberMotor.set(.8);
    }
    else{
      rightClimberMotor.set(0);
    }
    if (leftClimberMotor.getPosition().getValueAsDouble()> -185){
      leftClimberMotor.set(-.8);
    }
    else{
      leftClimberMotor.set(0);
    }
  }
  public void ClimberStop(){
    leftClimberMotor.set(0);
    rightClimberMotor.set(0);
  }

public boolean ClimberOnLimitSwitch(){
  if (rightLimitSwitch.get() == false && leftLimitSwitch.get() == false){
    return true;
  }
  else{
    return false;
  }
}

  @Override
  public void periodic() {
    if (rightLimitSwitch.get() == false){
      rightClimberMotor.setPosition(0);
    }
    if (leftLimitSwitch.get() == false){
      leftClimberMotor.setPosition(0);
    }
    SmartDashboard.putNumber("right CLimber motor rotation", rightClimberMotor.getPosition().getValueAsDouble());
     SmartDashboard.putNumber("left CLimber motor rotation", leftClimberMotor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Left Climer Motor Limit Switch", leftLimitSwitch.get());
     SmartDashboard.putBoolean("Right Climer Motor Limit Switch", rightLimitSwitch.get());
  }
}
