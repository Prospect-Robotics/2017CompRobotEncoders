package frc.team2813.robot.commands;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ToggleSolenoid extends InstantCommand {

	private Solenoid solenoid;
    public ToggleSolenoid(Solenoid solenoid) {
        super();
        this.solenoid = solenoid;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	solenoid.set(!solenoid.get());
    }

}
