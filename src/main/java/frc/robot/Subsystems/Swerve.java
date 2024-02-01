package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
//WALKER
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    

   

   
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

       
    
        if (Utils.isSimulation()) {
            startSimThread();
        }
    

 AutoBuilder.configureHolonomic(
    this::newPose, 
    this::resetPose,
    this::getSpeeds, 
    this::RobotRelative, 
    new HolonomicPathFollowerConfig( 
        new PIDConstants(5.0, 0.0, 0.0), 
        new PIDConstants(5.0, 0.0, 0.0),   4.5,  0.4, 
        new ReplanningConfig()), () -> {

var alliance = DriverStation.getAlliance();
if (alliance.isPresent()) { return alliance.get() == DriverStation.Alliance.Red; }
 return false;},
 this); }

    
 public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
        startSimThread();
    }
    
    AutoBuilder.configureHolonomic(
    this::newPose, 
    this::resetPose,
    this::getSpeeds, 
    this::RobotRelative, 
    new HolonomicPathFollowerConfig( 
        new PIDConstants(5.0, 0.0, 0.0), 
        new PIDConstants(5.0, 0.0, 0.0),   4.5,  0.4, 
        new ReplanningConfig()), () -> {

var alliance = DriverStation.getAlliance();
if (alliance.isPresent()) { return alliance.get() == DriverStation.Alliance.Red; }
 return false;},
 this); }


 public Pose2d newPose(){
        return this.m_odometry.getEstimatedPosition();
    }

  
    public void resetPose(Pose2d Pose){
        this.m_odometry.resetPosition(new Rotation2d(0), this.m_modulePositions.clone(), Pose);
    }

    public ChassisSpeeds getSpeeds(){
        return this.m_kinematics.toChassisSpeeds(this.m_cachedState.ModuleStates);
    }

    public void RobotRelative(ChassisSpeeds targetpostion){
        this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(targetpostion));
    }


    public Command applyRequest(Supplier<SwerveRequest> supplier) {
        return run(() -> this.setControl(supplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    
}
