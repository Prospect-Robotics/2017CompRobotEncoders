package frc.team2813.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team2813.robot.Robot;

/**
 *
 */
public class ReverseCamera extends InstantCommand {

    public ReverseCamera() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.server.setSource(Robot.server.getSource() == Robot.camera0 ? Robot.camera1 : Robot.camera0);
    	System.out.println("CAMERA/CONTROLS switch to " + (Robot.server.getSource() == Robot.camera0 ? "REAR" : "FRONT"));
    }

}
