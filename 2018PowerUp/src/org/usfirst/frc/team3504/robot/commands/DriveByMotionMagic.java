package org.usfirst.frc.team3504.robot.commands;

import org.usfirst.frc.team3504.robot.Robot;
import org.usfirst.frc.team3504.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveByMotionMagic extends Command {
	
	private double encoderTicks; //in sensor units
	private double headingUnits;
	private boolean resetPigeon;

	private WPI_TalonSRX leftTalon = Robot.chassis.getLeftTalon();
	private WPI_TalonSRX rightTalon = Robot.chassis.getRightTalon();
	
	private final static double DISTANCE_ERROR_THRESHOLD = 1000; //TODO tune (in encoder ticks)
	private final static double TURNING_ERROR_THRESHOLD = 6.0; //TODO tune (in degrees)

    public DriveByMotionMagic(double inches, double degrees) {
		encoderTicks = RobotMap.CODES_PER_WHEEL_REV * (inches / (RobotMap.WHEEL_DIAMETER * Math.PI));
		headingUnits = 10*degrees;
		resetPigeon = true;
		requires(Robot.chassis);
		System.out.println("DriveByMotionMagic: constructed");
    }
    
    public DriveByMotionMagic(double inches, double degrees, boolean reset) {
		encoderTicks = RobotMap.CODES_PER_WHEEL_REV * (inches / (RobotMap.WHEEL_DIAMETER * Math.PI));
		headingUnits = 10*degrees;
		resetPigeon = reset;
		requires(Robot.chassis);
		System.out.println("DriveByMotionMagic: constructed");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.chassis.setInverted(true);
    	System.out.println("DriveByMotionMagic: motors inverted");
    	Robot.chassis.configForMotionMagic();
    	System.out.println("DriveByMotionMagic: configured for motion magic");
    	if (resetPigeon) Robot.chassis.zeroSensors();
    	else Robot.chassis.zeroEncoder();
    	System.out.println("DriveByMotionMagic: sensors zeroed");
    	
    	System.out.println("DriveByMotionMagic encoder ticks + heading: " + encoderTicks + headingUnits);
    	System.out.println("DriveByMotionMagic initialized");
    	
		rightTalon.set(ControlMode.MotionMagic, 2 * encoderTicks, DemandType.AuxPID, headingUnits);
		leftTalon.follow(rightTalon, FollowerType.AuxOutput1);
		
		System.out.println("DriveByMotionMagic: instructions sent, started");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		//System.out.println("DriveByMotionMagic execute");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
  
    	if (!resetPigeon || headingUnits == 0)
    	{
    		double rightTicks = rightTalon.getSensorCollection().getQuadraturePosition();
    		double leftTicks = leftTalon.getSensorCollection().getQuadraturePosition();
    		double currentTicks = rightTicks + leftTicks;
    		System.out.println("sensor position: " + currentTicks);
    		System.out.println("target position: " + (2*encoderTicks));
    		double error = Math.abs((2*encoderTicks) - currentTicks);
    		System.out.println("Distance error: " + error);
    		return (error < DISTANCE_ERROR_THRESHOLD);
    	}
    	else
    	{
    		double currentHeading = Robot.chassis.getYaw();
    		double error = Math.abs((headingUnits/10) - currentHeading);
    		//System.out.println("Turning error: " + error);
    		return (error < TURNING_ERROR_THRESHOLD);
    	}
    	
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("DriveByMotionMagic: ended");
    	Robot.chassis.stop();
    	Robot.chassis.setInverted(false);
    	//Get out of follower mode? Undo some configs?
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("DriveByMotionMagic: interrupted, ending");
    	end();
    }
}
