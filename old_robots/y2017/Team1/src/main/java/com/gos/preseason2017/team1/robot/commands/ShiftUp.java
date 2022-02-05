package com.gos.preseason2017.team1.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import com.gos.preseason2017.team1.robot.subsystems.Shifters;
import com.gos.preseason2017.team1.robot.subsystems.Shifters.Speed;

/**
 *
 */
public class ShiftUp extends Command {
    private final Shifters m_shifters;

    public ShiftUp(Shifters shifters) {
        m_shifters = shifters;
        requires(m_shifters);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        m_shifters.shiftLeft(Speed.kHigh);
        m_shifters.shiftRight(Speed.kHigh);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        // The solenoid setting commands should complete immediately
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
        end();
    }
}