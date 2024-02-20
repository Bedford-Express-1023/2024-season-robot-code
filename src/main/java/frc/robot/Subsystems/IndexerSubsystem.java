// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonSRX indexerMotor = new TalonSRX(Constants.Indexer.INDEXER_CAN); //FIXME
   //private final DigitalInput indexerBeamBreak = new DigitalInput(Constants.Indexer.INDEXER_BEAM_BREAK_DIO); 
    
   // private boolean indexerBeamBreakValue;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {

  }

  public Command FeedShooter () {
    return runOnce(
      () -> {
      indexerMotor.set(ControlMode.PercentOutput, 1);
      });
    
  }

  public Command ReverseIndexer () {
    return runOnce(
      () -> {
      indexerMotor.set(ControlMode.PercentOutput, -.5);
      });
    
  }

  public Command IndexNote() {
    return runOnce(
      () -> {
      indexerMotor.set(ControlMode.PercentOutput, 0.1);
      });
  }

  public Command StopIndex() { 
    return runOnce(
      () -> {
         indexerMotor.set(ControlMode.PercentOutput,0);
      });
   
  }

  @Override
  public void periodic() {
  //  indexerBeamBreakValue = indexerBeamBreak.get();
    // This method will be called once per scheduler run
  }
}
