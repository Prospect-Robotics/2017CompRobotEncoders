package org.usfirst.frc2813.Robot4.commands;

import org.usfirst.frc2813.Robot4.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Rotate extends Command {
	private final double degrees,rotateSpeed;
	private double throttle;
	private double finalOrientation;
	private static final double LERP_START=60;//Degrees from target to start slowing down
	private static final double LERP_STOP=40;//Degrees from target to use minimum speed
	private static final double LERP_END=0.2;//minimum rotate speed to use
	private static final double MIN_DEG=0.01;//very small number to come to complete stop at
	
	public Rotate(double degrees,double rotateSpeed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	this.degrees=degrees;
    	this.rotateSpeed=rotateSpeed;
    	//this is like Python's self
    	
    	    	
    	throttle = rotateSpeed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.gyro.reset();
    	//assert (Robot.gyro.getAngle() == 0);
    	finalOrientation = Robot.gyro.getAngle() + degrees;//gyro.getAngle() should be 0
    }
    private double degreesRotated() {
    	return (Robot.gyro.getAngle());
    }
    private double calcThrottle(double deg) {//set throttle given degrees from target
    	if (deg<MIN_DEG) {//if at correct location, stop
    		return 0;
    	}
    	if (deg<=LERP_STOP) {//if through lerp period, min speed
    		return LERP_END;
    	}
    	if (deg>=LERP_START) {//if not at lerp period, given speed
    		return throttle;
    	}
    	return (deg-LERP_STOP)*(throttle-LERP_END)/(LERP_START-LERP_STOP)+LERP_END;//deceleration/linear interpolation code
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("degrees to go " +(finalOrientation-Robot.gyro.getAngle()));
    	double deg=degrees-degreesRotated();
    	double newThrottle=calcThrottle(deg);
    	System.out.println("throttle"+newThrottle);
    	Robot.driveTrain.mecanumDrive(0, 0, newThrottle, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("isFinished");
        return (degrees > 0 ? Robot.gyro.getAngle() >= finalOrientation : Robot.gyro.getAngle() <= finalOrientation);
        // a ? b : c is equivalent to Python's b if a else c
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
