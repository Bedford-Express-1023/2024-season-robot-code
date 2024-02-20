// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RotationalFeedForward;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor = new TalonFX(Constants.Intake.INTAKE_CAN);
  private final TalonFX intakePivotMotor = new TalonFX(Constants.Intake.INTAKE_PIVOT_CAN);
 private final CANcoder PivotCANCoder = new CANcoder(Constants.Intake.INTAKE_ENCODER_CAN);
  private final PositionVoltage intakePivotPosition = new PositionVoltage(0,0,true,0,1,false,false,false);
 double  motorPivotPower;
  public boolean intakeReadyToIndex;
  public double intakeUpSpeed;
  public double intakeDownSpeed;

  // fix pid values later
 private PIDController IntakePivotPID = new PIDController(.5, 0.0, 0);
  public static CANcoder leftFrontCANcoder = new CANcoder(4);
  public static CANcoder rightFrontCANcoder = new CANcoder(1);
  public static CANcoder leftBackCANcoder = new CANcoder(3);
  public static CANcoder rightBackCANcoder = new CANcoder(2);
  DigitalInput intakeBeamBreak = new DigitalInput(0);

 NeutralModeValue brake = NeutralModeValue.Brake;

 //public final RotationalFeedForward feedForward = new RotationalFeedForward(0,0.5,0, 0);
 

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

    configs.MotorOutput.NeutralMode = brake;

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    intakePivotMotor.getConfigurator().apply(configs);
 
    }

  public Command IntakeRun() {  
    return run(
      () -> {
    if(intakeBeamBreak.get() == true)
    {
          intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition));
        if(intakeAngle < (Constants.Intake.intakeDownPosition + .03) && intakeAngle > (Constants.Intake.intakeDownPosition -.03)){
       intakeMotor.set(-0.5);
       }
    }
    else if (intakeBeamBreak.get() == false){
        intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle ));
        intakeMotor.set(0);
       }
      
      });


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
    return run(
     
      () -> {
         intakeMotor.set(-0.5);
    //     if(intakeBeamBreak.get()==true)
    //     {
    // intakeMotor.set(-.5);
    //     }
    //     else if (intakeBeamBreak.get()==false)
    //     {
    //       intakeMotor.set(0);
    //     }
      });
    
  }

  public Command IntakeStop() {
    return runOnce(
    () -> {
     intakeMotor.set(0);
     });

   
  }

  public Command IntakePrepareToIndex() {
    //intakePivotMotor.setPosition(Constants.Intake.targetIntakePivotIndexAngle);
    
    return run(

    () -> {
   intakePivotMotor.set(IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle));
      // intakePivotMotor.set(-Math.abs(feedForward.calculate(motorPivotPower)));
     SmartDashboard.putNumber("up", motorPivotPower);
     intakeMotor.set(0);
     });
  }

  @Override
  public void periodic() {
    motorPivotPower = IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle);
  SmartDashboard.putNumber("down", IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition));
    SmartDashboard.putNumber("upAngle", IntakePivotPID.calculate(intakeAngle, Constants.Intake.targetIntakePivotIndexAngle));
    intakeAngle = PivotCANCoder.getAbsolutePosition().getValueAsDouble();

    if ((intakeAngle > Constants.Intake.targetIntakePivotIndexAngle - 0.05) && (intakeAngle < Constants.Intake.targetIntakePivotIndexAngle + 0.05)) {
      intakeReadyToIndex = true;
    }
    else {
      intakeReadyToIndex = false;
    }

   SmartDashboard.putBoolean("beam break", intakeBeamBreak.get());
    SmartDashboard.putNumber("Intake Angle", intakeAngle);
   // SmartDashboard.putNumber("up", IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeUpPosition));
   // SmartDashboard.putNumber("down", IntakePivotPID.calculate(intakeAngle, Constants.Intake.intakeDownPosition));
    // This method will be called once per scheduler run
  }
}