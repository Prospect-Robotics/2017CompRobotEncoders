package org.usfirst.frc2813.Robot4.commands;

import org.usfirst.frc2813.Robot4.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Rotate extends Command {
	private final double degrees,rotateSpeed;
	private double finalOrientation;

    public Rotate(double degrees,double rotateSpeed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	this.degrees=degrees;
    	this.rotateSpeed=rotateSpeed;
    	//this is like Python's self
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	finalOrientation = Robot.gyro.getAngle() + degrees;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println(finalOrientation-Robot.gyro.getAngle());
    	Robot.driveTrain.mecanumDrive(0, 0, rotateSpeed, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
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
