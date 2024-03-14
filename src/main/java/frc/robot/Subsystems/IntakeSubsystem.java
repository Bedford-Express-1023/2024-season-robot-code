// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX intakeMotor = new TalonFX(Constants.Intake.INTAKE_CAN);
  public final TalonFX intakePivotMotor = new TalonFX(Constants.Intake.INTAKE_PIVOT_CAN);
  public final CANcoder PivotCANCoder = new CANcoder(Constants.Intake.INTAKE_ENCODER_CAN);
  double motorPivotPower;
  public boolean intakeReadyToIndex;
  public boolean intakeBeamBreakValue;

  // fix pid values later
  public PIDController IntakePivotPID = new PIDController(.6, 0.01, 0);
  DigitalInput intakeBeamBreak = new DigitalInput(1);
  NeutralModeValue brake = NeutralModeValue.Brake;
  public double intakeAngle;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    configs.MotorOutput.NeutralMode = brake;

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    intakePivotMotor.getConfigurator().apply(configs);

  }

  public void IntakeRun() {
    if (intakeBeamBreak.get() == true) {
      intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition));
      intakeMotor.set(-0.5);
    } else {
      intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle));
      intakeMotor.set(0);
    }
  }

  public void IntakeDown() {
      intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition));
  }

  public void IntakeNote() {
    intakeMotor.set(-.5);
  }

  public void OutTake() {
    intakeMotor.set(.5);
  }

  public void IntakeStop() {
    intakeMotor.set(0);
  }

  public void IntakePrepareToIndex() {
    intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle));
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    motorPivotPower = IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle);
    intakeAngle = PivotCANCoder.getAbsolutePosition().getValueAsDouble();

    if ((intakeAngle > Constants.Intake.targetIntakePivotIndexAngle - 0.08)
        && (intakeAngle < Constants.Intake.targetIntakePivotIndexAngle + 0.08)) {
      intakeReadyToIndex = true;
    } else {
      intakeReadyToIndex = false;
    }
    intakeBeamBreakValue = intakeBeamBreak.get();
    SmartDashboard.putBoolean("beam break", intakeBeamBreak.get());
    SmartDashboard.putNumber("Intake Angle", intakeAngle);
  }
}