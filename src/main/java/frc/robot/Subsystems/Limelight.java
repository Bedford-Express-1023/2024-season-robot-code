// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandSwerveDrivetrain;

/** Add your docs here. */
public class Limelight extends SubsystemBase{
   //calling other subsystems
    private final LimelightHelpers limeLight = new LimelightHelpers();
 XboxController controller1 = new XboxController(0);
 

 public double leftXAxis;
 public double leftYAxis;
 public double RightXAxis;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   static double positionY = 3;
   static double positionX = -2;
    static double rotationPositon = -180;
    double forwardtpm = 0;
    double lefttpm = 0;
    double rotationtmp = 0;
    public double leftaxis = 0;
    // settign pid values
    PIDController pidX = new PIDController(.3, 0, 0);
    PIDController pidY = new PIDController(.3, 0, 0);
    PIDController pidRotaion = new PIDController(.003, 0, 0);

  
       
    @Override
    public void periodic() {
      // setting the controller1s to a variable
      double controllerLeftY = controller1.getLeftY();
      double controllerLeftX = controller1.getLeftX();
      double controllerRightX = controller1.getRightX();

   
        
        
        //getting the robots postion
        double[] botpose1 = LimelightHelpers.getBotPose("");
        double tx = botpose1[0];
        double ty = botpose1[1];
        double rz = botpose1[5];
    

        //setting the output of the robot based on the x and y to go to a setpoint
        forwardtpm = pidX.calculate(tx, positionX);
        lefttpm = pidY.calculate(ty, positionY);

        rotationtmp = pidRotaion.calculate(rz, rotationPositon);
      // setting the x-axis of the robot
        if ((controllerLeftX>.15 ) || (controllerLeftX <-.15)){
         leftXAxis = controllerLeftX;
         }
         else if(controller1.getXButton()){
        leftXAxis = lefttpm;
         }
         else{
            leftXAxis = 0;
         }

      // setting the left y-axis of the robot 
         if ((controllerLeftY>.15) || (controllerLeftY<-.15)) { 
           leftYAxis = controllerLeftY;
         }else if(controller1.getXButton()){
         leftYAxis =  forwardtpm;
         }
         else{
            leftYAxis = 0;
         }

         // setting the rotation axis of the robot
         if(controllerRightX>.15 ||(controllerRightX<-.15)){
       RightXAxis = controllerRightX;
         }
         else if(controller1.getXButton()){
            if (rotationPositon <rz){
         RightXAxis = -rotationtmp;
            }
            else{
               RightXAxis = rotationtmp;
            }
         }
         else{
            RightXAxis = 0 ;
         }
      //displaying the robot x and y
        SmartDashboard.putNumber("robotsXPosition", tx);
        SmartDashboard.putNumber("robotsYPosition", ty);
        SmartDashboard.putNumber("robotsRotaionPosition",rz);
       // putting the robots axis on to the dashboard
        SmartDashboard.putNumber("leftXaxis", leftXAxis);
        SmartDashboard.putNumber("leftYaxis", leftYAxis);
        SmartDashboard.putNumber("rightXaxis", RightXAxis);
        //displaying the outputs of the pid
        SmartDashboard.putNumber("forwardPower", forwardtpm);
        SmartDashboard.putNumber("leftPower", lefttpm);
        SmartDashboard.putNumber("rotaionPower", rotationtmp);
       //displaying controler controller1 posistion 
        SmartDashboard.putNumber("controlerleftY",controllerLeftY);
        SmartDashboard.putNumber("controlerleftX",controllerLeftX);
        SmartDashboard.putNumber("controlerRightX",controllerRightX);
        SmartDashboard.putBoolean("xbutton",controller1.getXButton());
        
        
    }
}
