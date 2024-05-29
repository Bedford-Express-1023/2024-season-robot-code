// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.lang.Character.Subset;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */

public class IntakeLimelight extends SubsystemBase {

   XboxController controller1 = new XboxController(0);
double intakeTX;
double intakeTY;
public double intakeRotation;
PIDController pidRotation = new PIDController(.0125, 0, 0);
     @Override
   public void periodic() {
   intakeTX = LimelightHelpers.getTX("limelight-intake");
   intakeTY = LimelightHelpers.getTY("limelight-intake");


      if(controller1.getAButton() == true) {
         pidRotation.setPID(0.02, 0.0, 0);
        intakeRotation  = pidRotation.calculate(intakeTX, 0);
      }
      else {
         pidRotation.setPID(.0, 0.0, 0);
         pidRotation.reset();

      }
if(intakeRotation > .10 && controller1.getAButton() ){
   intakeRotation = .10;
}
else if (intakeRotation < -.10 && controller1.getAButton()){
   intakeRotation = -.10;
}
SmartDashboard.putNumber("intaketx", intakeTX);
   }

}
