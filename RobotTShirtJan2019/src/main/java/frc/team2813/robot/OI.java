package frc.team2813.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team2813.robot.commands.*;

public class OI {
	
	public static Joystick joystick;
	public static final int JOYSTICK_ID = 0;
	
	public OI() {
		joystick = new Joystick(JOYSTICK_ID);
//		for(int id=0; id<=4; id++) {
		new JoystickButton(joystick, 1).whenPressed(new ToggleSolenoid(new Solenoid(0,1)));

		new JoystickButton(joystick, 2).whenPressed(new ToggleSolenoid(new Solenoid(0,7)));
			
//		new JoystickButton(joystick, 3).whenPressed(new ReverseCamera());
		
	}
	
}
