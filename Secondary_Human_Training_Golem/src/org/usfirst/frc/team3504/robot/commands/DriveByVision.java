package org.usfirst.frc.team3504.robot.commands;

import org.usfirst.frc.team3504.robot.Robot;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveByVision extends Command {

	NetworkTable table;
	
	private static final double MAX_CURVE = 0.5; //TODO: adjust
	private static final int IMAGE_WIDTH = 320;
	double[] defaultValue = new double[0];
	
	
	//width of X or Y in pixels when the robot is at the lift
	private static final double GOAL_WIDTH = 30; //TODO: test and change
	
	private static double encDist;
	private static double lastEncDist;
	
	private static final double MIN_DIST = 0.5;
	
    public DriveByVision() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Change more to Percent Vbus
    	Robot.chassis.driveLeftA.changeControlMode(TalonControlMode.PercentVbus);
    	Robot.chassis.driveRightA.changeControlMode(TalonControlMode.PercentVbus);
    	
    	table = NetworkTable.getTable("GRIP/myContoursReport");
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
	    double[] centerX = new double[2];
	    centerX = table.getNumberArray("centerX", defaultValue);
	    double[] centerY = new double[2];
	    centerY = table.getNumberArray("centerY", defaultValue);
	    double[] height = new double[2];
		height = table.getNumberArray("height", defaultValue);
	    double[] width = new double[2];
		width = table.getNumberArray("width", defaultValue);
		
		
		lastEncDist = encDist;
		encDist = Robot.chassis.getEncoderDistance();
		
		// the center of the x and y rectangles (the target)
    	double rotateValue;
    	if (centerX.length !=2){
    		rotateValue = 0;
    	} else {
    		double targetX = (centerX[0] + centerX[1])/2.0;
    		rotateValue = ((targetX - 160)/160)*MAX_CURVE;
    	}
    	SmartDashboard.putNumber("curve value", rotateValue);
    	
    	Robot.chassis.drive(.25, rotateValue); //TODO: change moveValue
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (Math.abs(encDist - lastEncDist) <= MIN_DIST);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
