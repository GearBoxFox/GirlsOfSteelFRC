package com.gos.outreach2016.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import com.gos.outreach2016.robot.subsystems.Manipulator;

/**
 *
 */
public class ShootBall extends Command {

    private final Manipulator m_manipulator;

    public ShootBall(Manipulator manipulator) {
        m_manipulator = manipulator;
        requires(m_manipulator);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        m_manipulator.shootBall();
        System.out.println("Shoot ball");
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}