// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor = new TalonFX(Constants.Intake.INTAKE_CAN);
  private final TalonFX intakePivotMotor = new TalonFX(Constants.Intake.INTAKE_PIVOT_CAN);
 private final CANcoder PivotCANCoder = new CANcoder(Constants.Intake.INTAKE_ENCODER_CAN);
  private final PositionVoltage intakePivotPosition = new PositionVoltage(0,0,true,0,1,false,false,false);

  public boolean intakeReadyToIndex;
  public double intakeUpSpeed;
  public double intakeDownSpeed;
 private PIDController IntakePivotPID = new PIDController(.75, 0, 0);

   //private final DigitalInput intakeBeamBreak = new DigitalInput(Constants.Indexer.INDEXER_BEAM_BREAK_DIO); 
   double intakeAngle;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot1.kP = .101;
    configs.Slot1.kI = .125;
    configs.Slot1.kD = .001;

    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
   
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    intakePivotMotor.getConfigurator().apply(configs);
    }

  public Command IntakeUp() {  
    return run(
      () -> {
           intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeUpPosition));
            SmartDashboard.putNumber("up", IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeUpPosition));
      });
    //intakePivotMotor.setPosition(Constants.Intake.intakeUpPosition);

  }

  public Command IntakeDown() {
     return run(
      () -> {
        intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition));
          SmartDashboard.putNumber("down", IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition));
      });
  //  intakePivotMotor.setPosition(Constants.Intake.intakeDownPosition)
  }
  public Command IntakePivotStop() {
    return run(
    () -> {
    intakePivotMotor.set(0);
     });

   
  }

  public Command IntakeNote() {
    return runOnce(
      () -> {
    intakeMotor.set(-1);
      });
    
  }

  public Command IntakeStop() {
    return runOnce(
    () -> {
     intakeMotor.set(0);
    intakePivotMotor.set(0);
     });

   
  }

  public Command IntakePrepareToIndex() {
    //intakePivotMotor.setPosition(Constants.Intake.targetIntakePivotIndexAngle);
    
    return runOnce(
    () -> {
 intakeMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle));
     });
  }

  @Override
  public void periodic() {

    intakeAngle = PivotCANCoder.getAbsolutePosition().getValueAsDouble();

    if ((intakeAngle > Constants.Intake.targetIntakePivotIndexAngle - 0.05) && (intakeAngle < Constants.Intake.targetIntakePivotIndexAngle + 0.05)) {
      intakeReadyToIndex = true;
    }
    else {
      intakeReadyToIndex = false;
    }

    SmartDashboard.putNumber("Intake Angle", intakeAngle);
   // SmartDashboard.putNumber("up", IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeUpPosition));
   // SmartDashboard.putNumber("down", IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition));
    // This method will be called once per scheduler run
  }
}