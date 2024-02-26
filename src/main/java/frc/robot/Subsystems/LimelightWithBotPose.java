// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LimelightWithBotPose extends SubsystemBase {

    @Override
    public void periodic() {
     //   LimelightHelpers.setPipelineIndex("",1);
        double[] botpose1 = LimelightHelpers.getBotPose("");
        double tx = botpose1[0];
        double ty = botpose1[1];
       double txoffset = 0;
       double tyoffset = 0;
       double DistanceAway = Math.hypot(tx-txoffset, ty-tyoffset);
       SmartDashboard.putNumber("Distance away from speaker",DistanceAway);


    }
}
