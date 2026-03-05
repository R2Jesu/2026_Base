// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import frc.robot.subsystems.DriveSubsystem; //replace with the subsytem(s) needed for your command
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


/** An example command that uses an example subsystem. */
public class R2Jesu_AlignCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    private boolean sideL;
    private PIDController yTarget;

    private Timer dontSeeTagTimer, stopTimer, overallTimer;

    private PIDController xControl = new PIDController(.1,0,0);
    private PIDController yControl = new PIDController(2.5,0,0);
    private PIDController zControl = new PIDController(.068,0,0);


// R2Jesu_AlignCommand aligns the robot with the apriltag.

    /**
     * Stops at distance away from apirl tag that is passed to the command
     * review with chris to determine if this should be more like shooter mode shoot with limelight
     * instead of mimicing align to tag from last year.
     * @param subsystem The subsystem used by this command.
     */
    public R2Jesu_AlignCommand(DriveSubsystem subsystem, boolean direction, PIDController target) {
        m_subsystem = subsystem;
        sideL = direction; // not sure if this is necessary vs we should be able to tell what side we are on
        yTarget = target; // pass in a distance from the limelight that you want to stop; est 30 in for climb



        // Use addRequirements() here to declare subsystem dependencies for each subsytem used
        addRequirements(subsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
