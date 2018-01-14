package org.usfirst.frc2813.TennisBallLauncher3.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Used in place of a hardware PWM splitter cable because, ya know, why fix it
 * in hardware when you can fix it for free in software? This could be used to
 * emulate a splitter over any kind of PWM speed controller, or any kind of
 * non-PWM speed controller, but why would you use this when you can tell your
 * CAN speed controller to be a follower of a master one?
 * 
 * @author Sean Worley <saworley39@yahoo.com>
 *
 */
public class PWMSplitter implements SpeedController, Sendable {

	private final List<SpeedController> controllers;
	private String name;
	private String subsystem;

	public PWMSplitter(SpeedController... controllers) {
		this.controllers = Arrays.asList(controllers);
	}

	@Override
	public void pidWrite(double output) {
		controllers.forEach((SpeedController c) -> c.pidWrite(output));
	}

	@Override
	public void set(double speed) {
		controllers.forEach((SpeedController c) -> c.set(speed));
	}

	@Override
	public double get() {
		return controllers.get(0).get();
	}

	@Override
	public void setInverted(boolean isInverted) {
		controllers.forEach((SpeedController c) -> c.setInverted(isInverted));
	}

	@Override
	public boolean getInverted() {
		return controllers.get(0).getInverted();
	}

	@Override
	public void disable() {
		controllers.forEach((SpeedController c) -> c.disable());
	}

	@Override
	public void stopMotor() {
		controllers.forEach((SpeedController c) -> c.stopMotor());
	}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return name;
	}

	@Override
	public void setName(String name) {
		this.name = name;
		controllers.stream().forEach((Object o) -> {
			if(o instanceof Sendable)
				((Sendable) o).setName(name);
		});
	}

	@Override
	public String getSubsystem() {
		// TODO Auto-generated method stub
		return subsystem;
	}

	@Override
	public void setSubsystem(String subsystem) {
		this.subsystem = subsystem;
		controllers.stream().forEach((Object o) -> {
			if(o instanceof Sendable)
				((Sendable) o).setSubsystem(subsystem);
		});
	}
	
	@Override
	public void setName(String subsystem, String name) {
		this.subsystem = subsystem;
		this.name = name;
		controllers.stream().forEach((Object o) -> {
			if(o instanceof Sendable) {
				((Sendable) o).setName(subsystem, name);
			}
		});
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		controllers.stream().forEach((Object o) -> {
			if(o instanceof Sendable)
				((Sendable) o).initSendable(builder);
		});
	}

}
