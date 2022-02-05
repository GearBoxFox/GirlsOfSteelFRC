package com.gos.preseason2017.team1.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import com.gos.preseason2017.team1.robot.subsystems.JawPiston;

/**
 *
 */
public class JawOut extends Command {
    private final JawPiston m_jaw;

    public JawOut(JawPiston jaw) {
        m_jaw = jaw;
        requires(m_jaw);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        m_jaw.pistonsOut();
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