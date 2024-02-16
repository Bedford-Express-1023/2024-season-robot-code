// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Limelight extends SubsystemBase{
private final FieldCentric drive = new FieldCentric(); 
   private final LimelightHelpers limeLight = new LimelightHelpers();
   XboxController controller1 = new XboxController(0);
   public double rotationtmp;
   double number = 0;
   PIDController pidRotation = new PIDController(0 , 0, 0);
   double kp = 0.0;
      
public Command RotateWithLimelight(){
  return runOnce(
    () -> {
      drive.withVelocityX(0) .withVelocityY(0).withRotationalRate(rotationtmp) ;
    });
}

    @Override
    public void periodic() {
   SmartDashboard.putNumber("number",number);
    // getting our pid numbers based on what we set them on the smart dashboard
    kp= SmartDashboard.getNumber("kp",0.0);
    double ki= SmartDashboard.getNumber("ki",0.0);
    double kd= SmartDashboard.getNumber("kd",0.0);
    // setting the pid to only run when we press y 
if (controller1.getYButton()== true)
   {
   pidRotation.setPID(kp, ki, kd);   
   
}else{

   pidRotation.setPID(0.0, 0.0, 0.0);   
   pidRotation.reset();
  
}
  
   // giving us a tolerance + or - .25 degrease.
    pidRotation.setTolerance(0.25);
   
        
        
   //getting april tags 4 and 7 tx values
   double Speakertx = LimelightHelpers.getTX("");
   rotationtmp = pidRotation.calculate(Speakertx, 0.0);
   SmartDashboard.putNumber("angle offset;", Speakertx);
             
    }
}
