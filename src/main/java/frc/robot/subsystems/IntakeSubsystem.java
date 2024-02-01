// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor = new TalonFX(Constants.Intake.INTAKE_CAN); //FIXME
  private final TalonFX intakePivotMotor = new TalonFX(Constants.Intake.INTAKE_PIVOT_CAN); //FIXME

  private double intakePivotMotorAngle;
  public boolean intakeReadyToIndex;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  public void IntakeUp() {
    intakePivotMotor.setPosition(Constants.Intake.intakeUpPosition);
  }

  public void IntakeDown() {
    intakePivotMotor.setPosition(Constants.Intake.intakeDownPosition);
  }

  public void IntakeNote() {
    intakeMotor.set(Constants.Intake.intakeSpeed);
  }

  public void IntakeStop() {
    intakeMotor.set(0);
  }

  public void IntakePrepareToIndex() {
    intakePivotMotor.setPosition(Constants.Intake.targetIntakePivotIndexAngle);
  }

  @Override
  public void periodic() {
    intakePivotMotorAngle = (intakePivotMotor.getPosition().getValueAsDouble() * 360);

    if ((intakePivotMotorAngle > Constants.Intake.minIntakePivotIndexAngle) && (intakePivotMotorAngle < Constants.Intake.maxIntakePivotIndexAngle)) {
      intakeReadyToIndex = true;
    }
    else {
      intakeReadyToIndex = false;
    }
    // This method will be called once per scheduler run
  }
}