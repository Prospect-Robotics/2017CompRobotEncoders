package org.usfirst.frc2813.TennisBallLauncher3.subsystems;

import java.util.Optional;

import org.usfirst.frc2813.TennisBallLauncher3.RobotMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/** Pneumatic components global to both shooters, i.e., the switch solenoid,
 * the master solenoid (if we ever get one), and the compressor (if we ever get one).
 *
 *
 * The terms "switch solenoid" and "master solenoid" aren't very clear.  The switch
 * solenoid is a dual-output single solenoid (as opposed to a double solenoid).
 * When power to it is off, air goes to one cylinder; when power is on, air goes
 * to the other cylinder.
 * 
 * The "master solenoid" is something on my wish list for this robot.  Turning it off
 * disconnects air to the switch solenoid, so that we can keep the cylinders at a driver-
 * configurable pressure (also on my wish list: pressure sensors).
 */
public class Pneumatics extends Subsystem {
	public final Optional<Compressor> compressor;
	public final Solenoid switchSolenoid;
	public final Optional<Solenoid> masterSolenoid;
    @SuppressWarnings("unused")
	public Pneumatics() {
    	switchSolenoid = new Solenoid(RobotMap.switchSolenoidPort);
    	if(RobotMap.masterSolenoidPort != -1)
    		masterSolenoid = Optional.of(new Solenoid(RobotMap.masterSolenoidPort));
    	else
    		masterSolenoid = Optional.empty();
    	
    	if(RobotMap.hasCompressor) 
    		compressor = Optional.of(new Compressor());
    	else
    		compressor = Optional.empty();
    }

    public void initDefaultCommand() {
        setDefaultCommand(null); // this subsystem doesn't really need an idle state
    }
    
    
}

