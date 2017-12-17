package org.usfirst.frc2813.Robot4.commands;

import org.usfirst.frc2813.Robot4.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class PrintOutEncoderValues extends InstantCommand {

    public PrintOutEncoderValues() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	System.out.println("Encoder 1 Value: " + Robot.driveTrain.quadratureEncoder1.get());
    	//System.out.println("Encoder 1 Distance:" + Robot.driveTrain.quadratureEncoder1.getDistance());
    	System.out.println("Encoder 2 Value: " + Robot.driveTrain.quadratureEncoder2.get());
    	//System.out.println("Encoder 2 Distance:" + Robot.driveTrain.quadratureEncoder2.getDistance());
    	System.out.println("Encoder 3 Value: " + Robot.driveTrain.quadratureEncoder3.get());
    	//System.out.println("Encoder 2 Distance:" + Robot.driveTrain.quadratureEncoder3.getDistance());
    	System.out.println("Encoder 4 Value: " + Robot.driveTrain.quadratureEncoder4.get());
    	//System.out.println("Encoder 4 Distance:" + Robot.driveTrain.quadratureEncoder4.getDistance());
    	
    }

}
