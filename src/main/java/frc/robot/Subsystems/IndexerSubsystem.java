// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

    public final TalonSRX indexerMotor = new TalonSRX(Constants.Indexer.INDEXER_CAN); //FIXME
    private final DigitalInput indexerBeamBreak = new DigitalInput(Constants.Indexer.INDEXER_BEAM_BREAK_DIO); 
    public boolean indexerBeamBreakValue;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    //TalonSRXConfiguration configs = new TalonSRXConfiguration();
    indexerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void FeedShooter () {
      indexerMotor.set(ControlMode.PercentOutput, 0.7);
  }

  public void FeedFastShooter () {
      indexerMotor.set(ControlMode.PercentOutput, 1);
  }
  public void ReverseIndexer () {
      indexerMotor.set(ControlMode.PercentOutput, -.3);
  }

  public void IndexNote() {
      indexerMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void StopIndex() { 
         indexerMotor.set(ControlMode.PercentOutput,0);
  }

  @Override
  public void periodic() {
  indexerBeamBreakValue = indexerBeamBreak.get();
  SmartDashboard.putBoolean("indexer beam break", indexerBeamBreak.get());
    // This method will be called once per scheduler run
  }
}
