package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Button;
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
    private final PreShooterSubsystem m_preShooter = new PreShooterSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();

    //Drivetrain
    private final TeleopArcadeDriveCommand m_teleopArcadeDriveCommand = new TeleopArcadeDriveCommand(m_drivetrain);
    private final FlywheelPrototypeTestCommand m_FlywheelPrototypeTestCommand = new FlywheelPrototypeTestCommand(m_drivetrain);
    private final PreShooterCommand m_preShooterCommand = new PreShooterCommand(m_preShooter);
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure all the control bindings
        configureControlBindings();
    }

    private void configureControlBindings() {
        
        //Configure control in the same order as the subsystems (alphabetical)

        //Drivetrain
        // m_teleopArcadeDriveCommand.setSuppliers(
        //     () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickLeftY), JOYSTICK_INPUT_EXPONENT),
        //     () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_INPUT_EXPONENT)
        //     );
        
        //for if we go back to prototypecommand for some reason
        //joystickDrive.getButton(JoystickF310.ButtonF310.A).toggleWhenPressed(m_FlywheelPrototypeTestCommand);

        //Just added a button to toggle pre shooter command
        joystickDrive.getButton(JoystickF310.ButtonF310.A).toggleWhenPressed(m_preShooterCommand);
        CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_FlywheelPrototypeTestCommand);
        CommandScheduler.getInstance().setDefaultCommand(m_preShooter, m_preShooterCommand);
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
