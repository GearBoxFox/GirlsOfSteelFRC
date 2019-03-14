package frc.robot.commands;

import frc.robot.GripPipelineListener;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveByVision extends Command {

	NetworkTable table;

	private static final double MAX_ANGULAR_VELOCITY = 1.0; // TODO: adjust
	// (rad/s) current
	// value works
	private static final int IMAGE_WIDTH = 320;
	private static final double IMAGE_CENTER = IMAGE_WIDTH / 2.0;
	double[] defaultValue = new double[0];
	private final int TIMEOUT = 8;
	private final int SLIPPING_VELOCITY = 850;
	private Timer tim;
	private double slowLinearVelocity = 22; // TODO: change (in/s)
	private double fastLinearVelocity = 28; // TODO: change (in/s)

	// width of X or Y in pixels when the robot is at the lift
	// private static final double GOAL_WIDTH = 30; //TODO: test and change

	private static final double WHEEL_BASE = 19; // distance between wheels (in)
	private static final int WHEEL_RADIUS = 3; // radius of wheel (in)

	public WPI_TalonSRX leftTalon = Robot.chassis.getLeftTalon();
	public WPI_TalonSRX rightTalon = Robot.chassis.getRightTalon();

	public DriveByVision() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.chassis);
		tim = new Timer();
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		// reset talon encoder reading to zero

		// not calling setupFPID because other PID values override
		leftTalon.set(ControlMode.Position, 0);
		rightTalon.set(ControlMode.Position, 0);

		// Change motor control to speed in the -1..+1 range
		//Robot.chassis.setSpeedMode();

		// tuned by janet and ziya on 2/20, overrides PID set in chassis method
		 leftTalon.config_kF(0, 0.22); // carpet on practice field
		 leftTalon.config_kP(0, 0.235);
		 rightTalon.config_kF(0, 0.2);
		 rightTalon.config_kP(0, 0.235);

		System.out.println("DriveByVision Initialized");

		tim.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double targetX;
		double height;

		synchronized(Robot.listener.cameraLock){
			targetX = Robot.listener.targetX;
			height = Robot.listener.height;
		}

		// the center of the x and y rectangles (the target)
		double goalAngularVelocity;
		if (targetX < 0) {
			goalAngularVelocity = 0;
			SmartDashboard.putBoolean("Bay in Sight", false);
		} else {
			//double targetX = (centerX[0] + centerX[1]) / 2.0;
			double error = (targetX - IMAGE_CENTER) / IMAGE_CENTER;
			goalAngularVelocity = error * MAX_ANGULAR_VELOCITY;
			SmartDashboard.putBoolean("Bay In Sight", true);
			SmartDashboard.putNumber("TargetX", targetX);
			SmartDashboard.putNumber("Height", height);
		}

		double goalLinearVelocity;
		if (height < 0 && tim.get() < 1)
			goalLinearVelocity = fastLinearVelocity;
		else if (height < 0) {
			goalLinearVelocity = slowLinearVelocity;
		} else if (height >= 52.0)
			goalLinearVelocity = slowLinearVelocity;
		else
			goalLinearVelocity = fastLinearVelocity;

		// right and left desired wheel speeds in inches per second
		double vRight = goalLinearVelocity - (WHEEL_BASE * goalAngularVelocity) / 2; // (in/s)
		double vLeft = goalLinearVelocity + (WHEEL_BASE * goalAngularVelocity) / 2;
		
		// right and left desired wheel speeds in RPM
		double angVRight = 75 * vRight / (2 * Math.PI * WHEEL_RADIUS); // (RPM)
		double angVLeft = 75 * vLeft / (2 * Math.PI * WHEEL_RADIUS);
	
		// send desired wheel speeds to Talon set to velocity control mode
		rightTalon.set(angVRight);
		leftTalon.set(-angVLeft);

		if (targetX >= 0){
			System.out.println("Number of Contours: " + 2/*centerX.length*/ + " Goal Linear Velocity: " + goalLinearVelocity
					+ " Goal Angular Velocity: " + goalAngularVelocity + " Timer: " + tim.get());
		}
		else {
			System.out.println("Number of Contours: " + "not 2" /*centerX.length*/ + " Goal Linear Velocity: " + goalLinearVelocity
					+ " Goal Angular Velocity: " + goalAngularVelocity + " Timer: " + tim.get());
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {

		return ((tim.get() > 1 && Math.abs(leftTalon.getSelectedSensorVelocity()) < SLIPPING_VELOCITY
				&& Math.abs(rightTalon.getSelectedSensorVelocity()) < SLIPPING_VELOCITY) || (tim.get() > TIMEOUT));
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("DriveByVision Finished");
		tim.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}