package org.usfirst.frc2813.Robot4.commands;

import org.usfirst.frc2813.Robot4.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ResetEncoders extends InstantCommand {

    public ResetEncoders() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.driveTrain.quadratureEncoder1.reset();
    	Robot.driveTrain.quadratureEncoder2.reset();
    	Robot.driveTrain.quadratureEncoder3.reset();
    	Robot.driveTrain.quadratureEncoder4.reset();
    }

}
