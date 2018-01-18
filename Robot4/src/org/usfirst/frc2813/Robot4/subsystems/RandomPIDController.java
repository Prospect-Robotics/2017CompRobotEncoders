package org.usfirst.frc2813.Robot4.subsystems;

import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class RandomPIDController implements PIDInterface {

	private double Kp, Ki, Kd, setpoint;
	private double integral, lastError;
	private final PIDSource src;
	private final PIDOutput out;
	private Thread m_thread;
	private boolean enabled;
	private boolean justStarted;

	public RandomPIDController(double p, double i, double d, PIDSource src, PIDOutput out) {
		setPID(p, i, d);
		this.src = src;
		this.out = out;
	}

	@Override
	public synchronized void setPID(double p, double i, double d) {
		Kp = p;
		Ki = i;
		Kd = d;
	}

	@Override
	public double getP() {
		return Kp;
	}

	@Override
	public double getI() {
		return Ki;
	}

	@Override
	public double getD() {
		return Kd;
	}

	@Override
	public synchronized void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}

	@Override
	public double getSetpoint() {
		return setpoint;
	}

	@Override
	public double getError() {
		// TODO Auto-generated method stub
		return src.pidGet() - setpoint;
	}

	@Override
	public void enable() {
		if (m_thread == null) {
			reset();
			m_thread = new Thread(this::calculate, "PID thread");
			m_thread.start();
		}
	}

	@Override
	public void disable() {
		if (m_thread != null) {
			m_thread.interrupt();
			m_thread = null;
		}
	}

	@Override
	public boolean isEnabled() {
		// TODO Auto-generated method stub
		return m_thread.isAlive();
	}

	@Override
	public synchronized void reset() {
		integral = 0;
		lastError = 0;
		justStarted = true;
	}

	protected void calculate() {
		calculate
	}

}
