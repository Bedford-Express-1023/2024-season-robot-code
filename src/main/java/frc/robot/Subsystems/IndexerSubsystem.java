// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

  public final TalonSRX indexerMotor = new TalonSRX(Constants.Indexer.INDEXER_CAN);
  private final DigitalInput indexerBeamBreak = new DigitalInput(Constants.Indexer.INDEXER_BEAM_BREAK_DIO);
  public boolean indexerBeamBreakValue;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    indexerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void FeedShooter() {
    indexerMotor.set(ControlMode.PercentOutput, 0.38);
  }

  public void FeedShooterFast() {
    indexerMotor.set(ControlMode.PercentOutput, 1);
  }

  public void ReverseIndexer() {
    indexerMotor.set(ControlMode.PercentOutput, -.6);
  }

  public void IndexNote() {
    indexerMotor.set(ControlMode.PercentOutput, 0.45);
  }

  public void StopIndex() {
    indexerMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    indexerBeamBreakValue = indexerBeamBreak.get();
    SmartDashboard.putBoolean("indexer beam break", indexerBeamBreak.get());
    // This method will be called once per scheduler run
  }
}
