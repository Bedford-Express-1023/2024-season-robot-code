// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;


public class Limelight extends SubsystemBase{
   CommandSwerveDrivetrain drivetrain;
   RobotContainer RobotContainer;
private final FieldCentric drive = new FieldCentric(); 
   private final LimelightHelpers limeLight = new LimelightHelpers();
   XboxController controller1 = new XboxController(0);
   public double rotationtmp;
   double number = 0;
   PIDController pidRotation = new PIDController(0.0125 , 0.00, 0);
   double kp = 0.0;
   public double botPositioning;
public Command RotateWithLimelight(){
  return run(
    () -> {
       pidRotation.setPID(.0125,0.005,0);   
     //  drive.withVelocityX(0).withVelocityY(0).withRotationalRate(rotationtmp);
    });
}
public Command StopRotatingWithLimelight(){
  return runOnce(
    () -> {
      pidRotation.reset();
   pidRotation.setPID(.0,0.0,0);  
    });
}
    @Override
    public void periodic() {
   SmartDashboard.putNumber("number",number);
    // getting our pid numbers based on what we set them on the smart dashboard
    kp= SmartDashboard.getNumber("kp",0.0);
   double ki= SmartDashboard.getNumber("ki",0.0);
   double kd= SmartDashboard.getNumber("kd",0.0);
//    double[] botPositioning = limeLight.getBotPose("");
//    double botX = botPositioning[0]; //position in x-axis of tbhe robot 
//    double botY = botPositioning[1]; //position in y-axis of the robot
//    double botRotation = botPositioning[5]; //the yaw or rotation of the robot 
//    // setting the pid to only run when we press y 
//    if (botRotation < 0)
//    {
//  botRotation = botRotation + 360;
//    }


if (controller1.getYButton()== true)
   {
   pidRotation.setPID(.0125,0.005,0);   
   
}else{ 
   pidRotation.setPID(.0,0.0,0);   
   pidRotation.reset();
  
}
  
   // giving us a tolerance + or - .25 degrease.
    pidRotation.setTolerance(0.25);
   
        
        
   //getting april tags 4 and 7 tx values
   double Speakertx = LimelightHelpers.getTX("");
   rotationtmp = pidRotation.calculate(Speakertx, 0.0);
   SmartDashboard.putNumber("angle offset;", Speakertx);
   SmartDashboard.putNumber("angle ofset power",rotationtmp);
    }
}
