package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class JoystickLeonardo extends GenericHID{

    public JoystickLeonardo(int joystickNumber)
    {
        super(joystickNumber);
    }   

    public JoystickButton getButton(int button) {
		return new JoystickButton(this, button);
	}

    public double getRawAxis(int axis) {
		
        return super.getRawAxis(axis);
	}

}
