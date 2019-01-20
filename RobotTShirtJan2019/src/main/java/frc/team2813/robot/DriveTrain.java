package frc.team2813.robot;
import frc.team2813.robot.commands.OIDrive;

import frc.team2813.robot.PWMSplitter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class DriveTrain extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final SpeedController speedController1 = new PWMSplitter(new Spark(0), new Spark(1), new Spark(2));
	private final SpeedController speedController2 = new PWMSplitter(new Spark(3), new Spark(4), new Spark(5));
	
    private final DifferentialDrive robotDrive = new DifferentialDrive(speedController1, speedController2);
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new OIDrive());     
    }
//    public static double getDistance() {
//    	return (ConfigureEncoderToTalon.getDefaultDistance(driveTrainLeft, PID_LOOP) + ConfigureEncoderToTalon.getDefaultDistance(driveTrainRight, PID_LOOP))/2;
//    }
    public void arcadeDrive(Joystick joystick1) {
		double z = joystick1.getX();//+joystick1.getTwist();
		// jack wants the other side of the robot to be the front, so invert the forwards/backwards axis.
		double x = -joystick1.getY();
		// square X and cube Z.
		robotDrive.arcadeDrive((x > 0 ? x*x : -x*x), z*z*z, false);
	}
}

