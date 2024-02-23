// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  //private final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
  //private final CurrentLimitsConfigs climberCurLimConfig = climberMotorConfig.CurrentLimits;

  private final TalonFX rightClimberMotor = new TalonFX(Constants.Climber.CLIMBER_RIGHT_CAN); //FIXME
  private final TalonFX leftClimberMotor = new TalonFX(Constants.Climber.CLIMBER_LEFT_CAN); //FIXME

  private StatusSignal<Double> rightClimberMotorStatorCurrent;
  private StatusSignal<Double> leftClimberMotorStatorCurrent;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    /* 
    climberCurLimConfig.StatorCurrentLimit = Constants.Climber.climberStatorCurrentLimit;
    climberCurLimConfig.StatorCurrentLimitEnable = false;
    climberMotorConfig.withCurrentLimits(climberCurLimConfig);
    rightClimberMotor.getConfigurator().apply(climberCurLimConfig); 
    leftClimberMotor.getConfigurator().apply(climberCurLimConfig); 
    */
  }

  public void ClimberUp() {
    rightClimberMotor.set(0.4);
    leftClimberMotor.set(-0.4);
  }

  public void ClimberDown() {
    rightClimberMotor.set(-0.4);
    rightClimberMotor.set(0.4 );
  }

  public void ClimberMaintainDown() {
    if (rightClimberMotorStatorCurrent.getValueAsDouble() > 5.5) {
      rightClimberMotor.set(-0.05);
    } 
    else if (rightClimberMotorStatorCurrent.getValueAsDouble() < 4.5) {
      rightClimberMotor.set(-0.2);
    }
    else {
      rightClimberMotor.set(-0.125);
    }
    
    if (leftClimberMotorStatorCurrent.getValueAsDouble() > 5.5) {
      leftClimberMotor.set(-0.05);
    } 
    else if (leftClimberMotorStatorCurrent.getValueAsDouble() < 4.5) {
      leftClimberMotor.set(-0.2);
    }
    else {
      leftClimberMotor.set(-0.125);
    }
  }

  @Override
  public void periodic() {
    rightClimberMotorStatorCurrent = rightClimberMotor.getStatorCurrent();
    leftClimberMotorStatorCurrent = leftClimberMotor.getStatorCurrent();

    // This method will be called once per scheduler run
  }
}
