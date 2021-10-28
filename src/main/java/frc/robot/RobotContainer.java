package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.JoystickF310.*;
import frc.robot.DriveUtil.*;

import static frc.robot.Constants.*;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    
    private final JoystickF310 joystickDrive = new JoystickF310(Ports.PORT_JOYSTICK_DRIVE);

    // Create subsystems
    private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();

    //Drivetrain
    private final TeleopArcadeDriveCommand m_teleopArcadeDriveCommand = new TeleopArcadeDriveCommand(m_drivetrain);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure all the control bindings
        configureControlBindings();
    }

    private void configureControlBindings() {
        
        //Configure control in the same order as the subsystems (alphabetical)

        //Drivetrain
        m_teleopArcadeDriveCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickLeftY), JOYSTICK_INPUT_EXPONENT),
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_INPUT_EXPONENT)
            );

        CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopArcadeDriveCommand);
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
