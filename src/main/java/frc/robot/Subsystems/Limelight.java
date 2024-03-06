// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
   XboxController controller1 = new XboxController(0);
   public double rotationtmp;
   PIDController pidRotation = new PIDController(0.0125, 0.00, 0);

   public void RotateWithLimelight() {
      pidRotation.setPID(.01, 0.002, 0);
   }

   public void StopRotatingWithLimelight() {
      pidRotation.reset();
      pidRotation.setPID(.0, 0.0, 0);
   }

   @Override
   public void periodic() {

      if (controller1.getYButton() == true) {
         pidRotation.setPID(.01, 0.002, 0);

      } else {
         pidRotation.setPID(.0, 0.0, 0);
         pidRotation.reset();

      }

      // giving us a tolerance + or - .25 degrease.
      pidRotation.setTolerance(0.25);

      // getting april tags 4 and 7 tx values
      double Speakertx = LimelightHelpers.getTX("");
      rotationtmp = pidRotation.calculate(Speakertx, 0.0);
      SmartDashboard.putNumber("angle offset;", Speakertx);
      SmartDashboard.putNumber("angle ofset power", rotationtmp);
   }
}
