// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX intakeMotor = new TalonFX(Constants.Intake.INTAKE_CAN);
  public final TalonFX intakePivotMotor = new TalonFX(Constants.Intake.INTAKE_PIVOT_CAN);
  public final CANcoder PivotCANCoder = new CANcoder(Constants.Intake.INTAKE_ENCODER_CAN);
  ArmFeedforward intakeFeedForward = new ArmFeedforward(0, 0.036132, 0, 0); 
  double intakeCanderZero;

  double motorPivotPower;
  public boolean intakeReadyToIndex;
  public boolean intakeBeamBreakValue;

  // fix pid values later
  public PIDController IntakePivotPID = new PIDController(1, 0.01, 0);
  DigitalInput intakeBeamBreak = new DigitalInput(1);
  NeutralModeValue brake = NeutralModeValue.Brake;
  public double intakeAngle;
  private final SendableChooser<String> zeroIntake = new SendableChooser<>();
  private final String ZeroIntakeOption = "zeroIntake";
    private final String DontZeroIntakeOption = "Don't Zero Intake";
private String zeroIntakeSelected;
double MagnetOffSet = .426;

    CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    CANcoderConfig.MagnetSensor.MagnetOffset = MagnetOffSet;
  PivotCANCoder.getConfigurator().apply(CANcoderConfig);

    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    configs.MotorOutput.NeutralMode = brake;

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    intakePivotMotor.getConfigurator().apply(configs);
  zeroIntake.addOption("Don't Zero Intake", DontZeroIntakeOption);
 zeroIntake.setDefaultOption("Don't Zero Intake", DontZeroIntakeOption);
 zeroIntake.addOption("zeroIntake", ZeroIntakeOption);
 SmartDashboard.putData("ZeroIntakeOption", zeroIntake);
 
  }

  public void IntakeRun() {
    if(intakeBeamBreakValue == true){

    
      intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition) 
        + intakeFeedForward.calculate(Constants.Intake.intakeDownPosition * 6.2832, 1));
        intakeMotor.set(-.5);
    }
    else{
            intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle ) 
        + intakeFeedForward.calculate(Constants.Intake.targetIntakePivotIndexAngle * 6.2832, 1));
      intakeMotor.set(0);
    }
  }

  public void IntakeDown() {
      intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition)
         + intakeFeedForward.calculate(Constants.Intake.intakeDownPosition * 6.2832, 1));
  }

  public void IntakeNote() {
    intakeMotor.set(-.7);
  }

  public void OutTake() {
    intakeMotor.set(.5);
  }

  public void IntakeStop() {
    intakeMotor.set(0);
  }

  public void IntakePrepareToIndex() {
    intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle)
                    + intakeFeedForward.calculate(Constants.Intake.targetIntakePivotIndexAngle * 6.2832, 1));
    intakeMotor.set(0);
  }

  public void IntakeZero() {
    intakePivotMotor.set(intakeFeedForward.calculate(0 * 6.2832, 1));
    intakeMotor.set(0);
  }
    public void IntakeZeroOnHardStop(){
    MagnetOffSet = MagnetOffSet + (.23 - intakeAngle );
    
    CANcoderConfig.MagnetSensor.MagnetOffset = MagnetOffSet;
  PivotCANCoder.getConfigurator().apply(CANcoderConfig);
  }
  public void IntakeZeroOnBumber(){
    MagnetOffSet = MagnetOffSet + (0-intakeAngle);
    
    CANcoderConfig.MagnetSensor.MagnetOffset = MagnetOffSet;
  PivotCANCoder.getConfigurator().apply(CANcoderConfig);
  }
  public void DontZero(){
    CANcoderConfig.MagnetSensor.MagnetOffset = MagnetOffSet;
  }


  @Override
  public void periodic() {
     intakeAngle = PivotCANCoder.getAbsolutePosition().getValueAsDouble();
     zeroIntakeSelected =zeroIntake.getSelected();

    //  if(zeroIntakeSelected == ZeroIntakeOption){
    //   PivotCANCoder.setPosition(0);
    //  }
    

    if ((intakeAngle > Constants.Intake.targetIntakePivotIndexAngle - 0.08)
        && (intakeAngle < Constants.Intake.targetIntakePivotIndexAngle + 0.08)) {
      intakeReadyToIndex = true;
    } else {
      intakeReadyToIndex = false;
    }
  if(MagnetOffSet > 1){
    MagnetOffSet = MagnetOffSet-1;
  }
    intakeBeamBreakValue = intakeBeamBreak.get();
    SmartDashboard.putBoolean("beam break", intakeBeamBreak.get());
    SmartDashboard.putNumber("Intake Angle", intakeAngle);
    SmartDashboard.putNumber("intake zero offset", MagnetOffSet);


  }
}