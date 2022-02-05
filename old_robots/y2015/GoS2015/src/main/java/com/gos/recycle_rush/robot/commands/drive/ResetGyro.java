package com.gos.recycle_rush.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import com.gos.recycle_rush.robot.subsystems.Chassis;

/**
 *
 */
public class ResetGyro extends Command {
    private final Chassis m_chassis;

    public ResetGyro(Chassis chassis) {
        m_chassis = chassis;
        requires(m_chassis);
    }

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        m_chassis.resetGyro();
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