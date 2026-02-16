// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.math.geometry.Pose2d;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final Field2d ourfield = new Field2d();

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }



  public Pigeon2 getPigeon() {
    Pigeon2 pigeon2 = m_robotContainer.m_robotDrive.getPigeon2();
    return pigeon2;
  }
    
 @Override
  public void robotInit() {
    SmartDashboard.putData("Field", ourfield);
    getPigeon().setYaw(0.0);
    }

  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    LimelightHelpers.PoseEstimate myLimelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
   if (myLimelightPose.tagCount >= 2) {
        m_robotContainer.m_robotDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        m_robotContainer.m_robotDrive.addVisionMeasurement(myLimelightPose.pose, myLimelightPose.timestampSeconds);
      }
    ourfield.setRobotPose(m_robotContainer.m_robotDrive.getState().Pose);

//based on the video, we are putting in logic to reset our robot codes based on robot position
    double omegaRps = Units.degreesToRotations(m_robotContainer.m_robotDrive.getTurnRate());
    var llMeasurement = LimelightHelpers.getBotPose_wpiBlue("limelight");
    
//original example
//    if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
//      m_robotContainer.m_robotDrive.resetOdometry(llMeasurement.pose); 
//our revised attempt  CHRIS TONEY PLS HELP - trying to reset robot pose to the limelights pose
//    if (llMeasurement != null && LimelightHelpers.getTargetCount("limelight") > 0 && Math.abs(omegaRps) < 2.0) {
//      m_robotContainer.m_robotDrive.setRobotPose(llMeasurement);
    

  
//odometry aiming and ranging: docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-and-ranging

    CommandScheduler.getInstance().run();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    SmartDashboard.putString("Choice", m_autonomousCommand.toString());
    SmartDashboard.putNumber("Tag Count", myLimelightPose.tagCount);
    //SmartDashboard.putNumber("Pigeonyaw", Rotation2d.fromDegrees(m_robotContainer.m_robotDrive.getPigeon2().getYaw().getValueAsDouble()).getDegrees());
    SmartDashboard.putNumber("pigeon2 yaw", Math.floorMod((int) getPigeon().getYaw().getValueAsDouble(), 360));
  }
  

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}

