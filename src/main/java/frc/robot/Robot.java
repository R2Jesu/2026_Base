// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.math.geometry.Pose2d;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final Field2d ourfield = new Field2d();

  private final RobotContainer m_robotContainer;
  double omegaRPS; 

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
    m_robotContainer.m_robotDrive.getPigeon2().reset();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("SwerveDrive");
  
      builder.addDoubleProperty("Front Left Angle", () -> m_robotContainer.m_robotDrive.getState().ModuleStates[0].angle.getRadians(), null);
      builder.addDoubleProperty("Front Left Velocity", () -> m_robotContainer.m_robotDrive.getState().ModuleStates[0].speedMetersPerSecond, null);
  
      builder.addDoubleProperty("Front Right Angle", () -> m_robotContainer.m_robotDrive.getState().ModuleStates[1].angle.getRadians(), null);
      builder.addDoubleProperty("Front Right Velocity", () -> m_robotContainer.m_robotDrive.getState().ModuleStates[1].speedMetersPerSecond, null);
  
      builder.addDoubleProperty("Back Left Angle", () -> m_robotContainer.m_robotDrive.getState().ModuleStates[2].angle.getRadians(), null);
      builder.addDoubleProperty("Back Left Velocity", () -> m_robotContainer.m_robotDrive.getState().ModuleStates[2].speedMetersPerSecond, null);
  
      builder.addDoubleProperty("Back Right Angle", () -> m_robotContainer.m_robotDrive.getState().ModuleStates[3].angle.getRadians(), null);
      builder.addDoubleProperty("Back Right Velocity", () -> m_robotContainer.m_robotDrive.getState().ModuleStates[3].speedMetersPerSecond, null);
  
      builder.addDoubleProperty("Robot Angle", () -> m_robotContainer.m_robotDrive.getState().Pose.getRotation().getRadians(), null);
      }
    });
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    LimelightHelpers.SetRobotOrientation(Constants.kLimelightName, m_robotContainer.m_robotDrive.getState().RawHeading.getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate myLimelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
   if (myLimelightPose != null && myLimelightPose.tagCount >= 1 && myLimelightPose.avgTagDist < 6.0 && myLimelightPose.tagSpan > 0.1) {
        m_robotContainer.m_robotDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.9, 0.9, 0.9));
        m_robotContainer.m_robotDrive.addVisionMeasurement(myLimelightPose.pose, myLimelightPose.timestampSeconds);
      }
    ourfield.setRobotPose(m_robotContainer.m_robotDrive.getState().Pose);
    omegaRPS = Units.degreesToRotations(m_robotContainer.m_robotDrive.getTurnRate());
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    double distance = poseEstimate.avgTagDist; 
//odometry aiming and ranging: docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-and-ranging

/*     SmartDashboard.putString("Choice", m_autonomousCommand.toString());
    SmartDashboard.putNumber("Tag Count", myLimelightPose.tagCount);
    SmartDashboard.putNumber("Pigeonyaw", m_robotContainer.m_robotDrive.getState().RawHeading.getDegrees());
    SmartDashboard.putNumber("pigeon2 yaw", Math.floorMod((int) getPigeon().getYaw().getValueAsDouble(), 360));
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("omegaRPS", Math.abs(omegaRPS));
    SmartDashboard.putNumber("Robotx", m_robotContainer.m_robotDrive.getState().Pose.getX());
    SmartDashboard.putNumber("Roboty", m_robotContainer.m_robotDrive.getState().Pose.getY());
    SmartDashboard.putNumber("Robotrotation", m_robotContainer.m_robotDrive.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Limelightx", poseEstimate.pose.getX());
    SmartDashboard.putNumber("Limelighty", poseEstimate.pose.getY());
    SmartDashboard.putNumber("Limelightrotation", poseEstimate.pose.getRotation().getDegrees()); */
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

