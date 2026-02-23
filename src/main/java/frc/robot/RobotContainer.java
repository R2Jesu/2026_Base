// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;



import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.R2Jesu_ShooterSubsystem;
import frc.robot.commands.ShooterModeShootWithLimelight;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final DriveSubsystem m_robotDrive = TunerConstants.createDrivetrain();
    public final R2Jesu_ShooterSubsystem m_shooterSubsystem = new R2Jesu_ShooterSubsystem();
    // need to understand why drivetain doesnt = new DriveSubsystem();

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(5.0); // 3 m/s^2
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(5.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Math.PI); // rad/s^2

    public RobotContainer() {
        registerAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void registerAutoCommands(){

  NamedCommands.registerCommand("ppShoot", Commands.print("Command to shoot preloaded balls"));
  NamedCommands.registerCommand("ppHang", new SequentialCommandGroup(Commands.print("Command to hang"), Commands.waitSeconds(1), Commands.print("Command to release")));
  

    
  }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_robotDrive.setDefaultCommand(
            // m_robotDrive will execute this command periodically
            m_robotDrive.applyRequest(() ->
                drive.withVelocityX(yLimiter.calculate(-joystick.getRightY())) // Drive forward with negative Y (forward)
                    .withVelocityY(xLimiter.calculate(-joystick.getRightX())) // Drive left with negative X (left)
                    //.withRotationalRate(rotLimiter.calculate(-joystick.getLeftX())) // Drive counterclockwise with negative X (left)
                    .withRotationalRate(-joystick.getLeftX()) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_robotDrive.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(m_robotDrive.applyRequest(() -> brake));
        joystick.b().whileTrue(m_robotDrive.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(m_robotDrive.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(m_robotDrive.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(m_robotDrive.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(m_robotDrive.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(m_robotDrive.runOnce(() -> m_robotDrive.seedFieldCentric()));

        m_robotDrive.registerTelemetry(logger::telemeterize);

        //Driver Buttons and such
        joystick.rightTrigger().whileTrue(new ShooterModeShootWithLimelight(m_shooterSubsystem, m_robotDrive,
            joystick));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
