// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.LimelightHelpers;

/** An DriveSubsystem command that uses an DriveSubsystem subsystem. */
public class R2Jesu_AlignHangCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private boolean sideL;
  private Timer dontSeeTagTimer, stopTimer, overallTimer;
  private PIDController xControl = new PIDController(.1, 0, 0);
  private PIDController yControl = new PIDController(2.5, 0, 0);  
  private PIDController zControl = new PIDController(.068, 0, .0);

  private final DriveSubsystem m_subsystem;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed set to 1 initially - press fn f12 to see setting
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity - change to rotate faster
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors - can change to velocity 
  /**
   * Creates a new SwerveCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public R2Jesu_AlignHangCommand(DriveSubsystem subsystem, boolean direction) {
    m_subsystem = subsystem; 
    sideL=direction;

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.overallTimer = new Timer();
    this.overallTimer.start();
    zControl.setSetpoint(0.0);
    zControl.setTolerance(.2);

    if (sideL) {
      yControl.setSetpoint(-.175);
    }
    else {
      yControl.setSetpoint(.175);
    }
    yControl.setTolerance(.1);

    xControl.setSetpoint(6.0);
    xControl.setTolerance(.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getFiducialID("limelight") != 0) {
      dontSeeTagTimer.reset();
      double[] positions = LimelightHelpers.getCameraPose_TargetSpace("limelight");

     // double xSpeed = -(xControl.calculate(DriveSubsystem.distInIn, xControl.getSetpoint()));
      double xSpeed = -(xControl.calculate(LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.kLimelightName).avgTagDist, xControl.getSetpoint()));
/*       if (DriveSubsystem.distInIn <= 6.0) {
        xSpeed = 0;
      } */
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yControl.calculate(positions[0]);
      SmartDashboard.putNumber("yspeed", ySpeed);
      double rotValue = -zControl.calculate(positions[4]);
      SmartDashboard.putNumber("zspeed", rotValue);
      SmartDashboard.putNumber("tx:", LimelightHelpers.getTX("limelight"));
      

      this.drive(xSpeed, ySpeed, rotValue);
      
      //m_subsystem.drive(new Translation2d(yControl.getError() < 0.2 ? xSpeed : 0, ySpeed), rotValue, false);
      //m_subsystem.drive(new Translation2d(xControl.atSetpoint() ? 0 : xSpeed, ySpeed), rotValue, false);
      //m_subsystem.drive(new Translation2d(0, ySpeed), rotValue, false);


      if (!zControl.atSetpoint() ||
          !yControl.atSetpoint() ||
          !xControl.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      this.drive(0, 0, 0);
      System.out.println("hit set");
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(1.0) ||
        stopTimer.hasElapsed(0.3) || overallTimer.hasElapsed(2.0);
  } 


  private void drive(double x, double y, double rot){

    m_subsystem.applyRequest(() ->
                drive.withVelocityX(x) // Drive forward with negative Y (forward)
                    .withVelocityY(y) // Drive left with negative X (left)
                    //.withRotationalRate(rotLimiter.calculate(-joystick.getLeftX())) // Drive counterclockwise with negative X (left)
                    .withRotationalRate(rot) // Drive counterclockwise with negative X (left)
            );
  }
}
