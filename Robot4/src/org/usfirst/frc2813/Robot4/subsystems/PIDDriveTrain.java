package org.usfirst.frc2813.Robot4.subsystems;

import org.usfirst.frc2813.Robot4.Robot;
import org.usfirst.frc2813.Robot4.RobotMap;
import org.usfirst.frc2813.Robot4.commands.OIDrive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 *
 */
public class PIDDriveTrain extends Subsystem implements MotorSafety {
	protected static final double ROTATE_FULL_SPEED_DEGREES_PER_SECOND = 200;
	private final MotorSafetyHelper m_safetyHelper = new MotorSafetyHelper(this);
	private final MecanumDrive m_driveTrain = RobotMap.driveTrainRobotDrive41;

	private final PIDSource m_pidSource = new PIDSource() {

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			Robot.gyro.setPIDSourceType(pidSource);
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return Robot.gyro.getPIDSourceType();
		}

		@Override
		public double pidGet() {
			return Robot.gyro.getRate() / ROTATE_FULL_SPEED_DEGREES_PER_SECOND;
		}

	};

	public final PIDController m_controller = new PIDController(0, 0, 0, m_pidSource, this::usePIDOutput);
	private double xSpeed, ySpeed;
	private boolean fieldOriented;
	

	public final Encoder quadratureEncoder2 = RobotMap.driveTrainQuadratureEncoder2;
	public final Encoder quadratureEncoder3 = RobotMap.driveTrainQuadratureEncoder3;
	public final Encoder quadratureEncoder4 = RobotMap.driveTrainQuadratureEncoder4;O

	// Initialize your subsystem here
	public PIDDriveTrain() {
		m_controller.disable();
		addChild("Steer correction", m_controller);
		m_driveTrain.setSafetyEnabled(false);
		addChild(m_driveTrain);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new OIDrive());
	}

	protected void usePIDOutput(double output) {
		m_driveTrain.driveCartesian(xSpeed, ySpeed, output, fieldOriented ? Robot.gyro.getAngle() : 0);
	}

	@Override
	public void setExpiration(double timeout) {
		m_safetyHelper.setExpiration(timeout);
	}

	@Override
	public double getExpiration() {
		return m_safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		return m_safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		// I'm guessing that most of the time this is going to be what ends up turning the motors off.
		m_controller.disable();
		m_driveTrain.stopMotor();
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		m_safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
		return m_safetyHelper.isSafetyEnabled();
	}

	public void mecanumDrive(double x, double y, double rotate, boolean fieldOriented) {
		xSpeed = x;
		ySpeed = y;
		this.fieldOriented = fieldOriented;
		m_controller.setSetpoint(rotate);
		m_controller.enable();
		m_safetyHelper.feed();
	}

	@Override
	public String getDescription() {
		// Eh...whatever.  This'll do.
		return getSubsystem();
	}
}
