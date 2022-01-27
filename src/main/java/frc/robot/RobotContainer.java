package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureControlBindings() {
        
        //Configure control in the same order as the subsystems (alphabetical)

        //Drivetrain
        // m_teleopArcadeDriveCommand.setSuppliers(
        //     () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickLeftY), JOYSTICK_INPUT_EXPONENT),
        //     () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_INPUT_EXPONENT)
        //     );

        
        // m_FlywheelPrototypeTestCommand.setSupplier(
        //     () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightY), JOYSTICK_INPUT_EXPONENT);
        //     );
        joystickDrive.getButton(ButtonF310.A).toggleWhenPressed(m_FlywheelPrototypeTestCommand);
        
        //CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, /*m_teleopArcadeDriveCommand*/m_FlywheelPrototypeTestCommand);
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
