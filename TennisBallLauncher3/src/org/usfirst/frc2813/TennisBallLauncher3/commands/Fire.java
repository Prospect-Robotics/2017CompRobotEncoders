package org.usfirst.frc2813.TennisBallLauncher3.commands;

import org.usfirst.frc2813.TennisBallLauncher3.Robot;
import org.usfirst.frc2813.TennisBallLauncher3.RobotMap;
import org.usfirst.frc2813.TennisBallLauncher3.subsystems.Cannon;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class Fire extends TimedCommand {

	private Cannon cannon;
	public static double SOLENOID_ON_TIME = 0.25D;
	//private Solenoid solenoid = new Solenoid(RobotMap.solenoidChanger);
    public Fire(Cannon cannon) {
    	super(SOLENOID_ON_TIME);
        requires(cannon);
        requires(Robot.getInstance().pneumatics);
        this.cannon=cannon;
    }

    // Called as the command starts executing
    protected void initialize() {
    	// disconnect the tank while firing:
    	//solenoid.set(!solenoid.get());
    	cannon.setPressurizing(false);
    	cannon.solenoid.set(true);
    }

    // Called as the command stops executing (after the set delay or when it is canceled)
    protected void end() {
    	cannon.solenoid.set(false);
    	// repressurize the cylinder
    	cannon.setPressurizing(true);
    }
}
