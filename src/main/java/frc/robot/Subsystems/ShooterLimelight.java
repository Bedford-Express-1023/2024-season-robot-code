// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterLimelight extends SubsystemBase {
   XboxController controller1 = new XboxController(0);
   public double rotationtmp;
   double rotationTolerance;
   public PIDController pidRotation = new PIDController(.0125, 0, 0);//0.5, 0.5, 0.05);//0.0125, 0.00, 0);
   LimelightHelpers intakeLImelight = new LimelightHelpers();
   double Speakertx;
   double feederRotationLine;
   public void RotateWithLimelight() {
      pidRotation.setPID(.01, 0.002, 0);
      rotationtmp = pidRotation.calculate(Speakertx, 0.0);
   }

   public void StopRotatingWithLimelight() {
      pidRotation.reset();
      pidRotation.setPID(.0, 0.0, 0);
   }
   public boolean IsRotated(){
  if(MathUtil.isNear(0.1, Speakertx, 3)){
   return true;
  }
  else{
   return false;
  }
   }
   public boolean AprilTagSeen(){
  if(Speakertx < .1 || Speakertx > -.1)
  {
   return true;
  }
  else{
   return false;
  }
   }
   @Override
   public void periodic() {
      double distanceWithLimelight =  Math.tan((Math.toRadians(LimelightHelpers.getTY("limelight-shooter") + 29)) / 45.5);
      if (controller1.getYButton() == true) {
         pidRotation.setPID(0.02, 0.0, 0);
     rotationtmp = pidRotation.calculate(Speakertx, 0.0);
      }
      else if(controller1.getXButton() == true) {
         pidRotation.setPID(0.02, 0.0, 0);
         rotationtmp = pidRotation.calculate(Speakertx, feederRotationLine);
      }
      else {
         pidRotation.setPID(.0, 0.0, 0);
         pidRotation.reset();

      }
if(rotationtmp > .15 && controller1.getXButton() ){
   rotationtmp = .15;
}
else if (rotationtmp < -.15 && controller1.getXButton()){
   rotationtmp = -.15;
}

      // giving us a tolerance + or - .25 degrease.
      pidRotation.setTolerance(0.25);
    
      // getting april tags 4 and 7 tx values
      Speakertx = LimelightHelpers.getTX("limelight-shooter");
feederRotationLine = -4000 * distanceWithLimelight +18; 

if (distanceWithLimelight > .0119){
   rotationTolerance = 5;
 }
 else{
 rotationTolerance = 1.5;
 }
 

      SmartDashboard.putNumber("limelight rotation power",rotationtmp);
      SmartDashboard.putNumber("Speaker tx", Speakertx);
      SmartDashboard.putNumber("feeder Rotation offset",feederRotationLine);
   }
}
