// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private final CurrentLimitsConfigs climberCurLim = new CurrentLimitsConfigs();
  private final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();

  private final TalonFX rightClimberMotor = new TalonFX(Constants.Climber.CLIMBER_RIGHT_CAN); //FIXME
  private final TalonFX leftClimberMotor = new TalonFX(Constants.Climber.CLIMBER_LEFT_CAN); //FIXME
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotorConfig.withCurrentLimits(climberCurLim);
    rightClimberMotor.getConfigurator().apply(climberCurLim); 
  }

  public void ClimberUp() {
    rightClimberMotor.setPosition(Constants.Climber.climberUpPosition);
    leftClimberMotor.setPosition(Constants.Climber.climberUpPosition);
  }

  public void ClimberDown() {
    rightClimberMotor.setPosition(Constants.Climber.climberDownPosition);
    leftClimberMotor.setPosition(Constants.Climber.climberDownPosition);
  }

  public void ClimberMaintainDown() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
