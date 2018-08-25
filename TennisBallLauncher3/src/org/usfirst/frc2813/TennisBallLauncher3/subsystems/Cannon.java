package org.usfirst.frc2813.TennisBallLauncher3.subsystems;

import org.usfirst.frc2813.TennisBallLauncher3.Robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Cannon extends Subsystem {
	private boolean whichSide;
	public Solenoid solenoid;
	public Cannon(boolean whichSide, int solenoidPort) {
		this.whichSide=whichSide;
		solenoid = new Solenoid(solenoidPort);
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    protected void initDefaultCommand() {
        setDefaultCommand(null);
    }
    
    /**
     * Switch the master switch solenoid to our side to repressurize the cylinder.
     * Do this after firing.
     * @param on 
     */
	public void setPressurizing(boolean on) {
		Pneumatics pneumatics = Robot.getInstance().pneumatics; // syntactic sugar
		if(on) {
			pneumatics.switchSolenoid.set(whichSide);
			pneumatics.masterSolenoid.ifPresent((Solenoid solenoid) -> solenoid.set(true));
		} else {
			if(pneumatics.masterSolenoid.isPresent()) {
				pneumatics.masterSolenoid.get().set(false);
			} else {
				pneumatics.switchSolenoid.set(!whichSide);
			}
		}
		if(on)
			pneumatics.compressor.ifPresent(Compressor::start); // XXX necessary?
	}
    
    
}

