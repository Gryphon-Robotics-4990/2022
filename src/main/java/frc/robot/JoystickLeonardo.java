package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import frc.robot.JoystickF310.ButtonF310;

public class JoystickLeonardo extends GenericHID{
    private static final double JOYSTICKLEONARDO_AXIS_DEADBAND = 0.05;

    public JoystickLeonardo(int joystickNumber)
    {
        super(joystickNumber);
    }   

    public Button getButton(ButtonLeonardo button) {
		return new JoystickButton(this, button.ordinal());
	}

    public Button getTempButton()
    {
        return new JoystickButton(this, 0);
    }

	public Button getButton(POVLeonardo button) {
		return new POVButton(this, button.ordinal() * 45);
	}

	// public double getRawAxis(AxisLeonardo axis) {
	// 	if (axis == AxisLeonardo.JoystickLeftY || axis == AxisLeonardo.JoystickRightY) return -1 * this.getRawAxis(axis.ordinal());
	// 	return this.getRawAxis(axis.ordinal());
	// }

	public double getRawAxis(int axis) {
		return Math.abs(super.getRawAxis(axis)) < JOYSTICKLEONARDO_AXIS_DEADBAND ? 0 : super.getRawAxis(axis);
	}

	public static enum POVLeonardo {
		Top, TopRight, Right, BottomRight, Bottom, BottomLeft, Left, TopLeft
	}

	public static enum ButtonLeonardo {
		None, A
	}

	public static enum AxisLeonardo {
		JoystickLeftX, JoystickLeftY, Throttle
    }
}
