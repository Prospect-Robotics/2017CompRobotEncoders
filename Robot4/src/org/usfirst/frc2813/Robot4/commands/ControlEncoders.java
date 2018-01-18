package org.usfirst.frc2813.Robot4.commands;

import org.usfirst.frc2813.Robot4.Robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ControlEncoders extends Command {
	Encoder myEncoder1;
	Encoder myEncoder2;
	Encoder myEncoder3;
    public ControlEncoders(Encoder myEncoder1, Encoder myEncoder2, Encoder myEncoder3) {
    	requires(Robot.driveTrain);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	while(true) {
    		if(Math.abs(myEncoder3.getDistance()) <= Math.abs(myEncoder2.getDistance()) - 3 || Math.abs(myEncoder3.getDistance()) <= Math.abs(myEncoder1.getDistance()) - 3) {
    			new Rotate(-5,0.5);
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
