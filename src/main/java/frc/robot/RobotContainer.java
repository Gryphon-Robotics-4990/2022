package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.vision.VisionController;
import frc.robot.JoystickF310.*;
import frc.robot.DriveUtil.*;

import static frc.robot.Constants.*;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    
    private final JoystickF310 joystickDrive = new JoystickF310(Ports.PORT_JOYSTICK_DRIVE);
    private final JoystickF310 joystickOperator = new JoystickF310(Ports.PORT_JOYSTICK_OPERATOR);

    // Create subsystems
    private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final TurretSubsystem m_turret = new TurretSubsystem();
    //Create Commands
    private final TeleopArcadeDriveCommand m_teleopArcadeDriveCommand = new TeleopArcadeDriveCommand(m_drivetrain);
    private final FlywheelPrototypeTestCommand m_flywheelPrototypeTestCommand = new FlywheelPrototypeTestCommand(m_drivetrain);
    private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intake);
    private final TurretManualCommand m_turretManualCommand = new TurretManualCommand(m_turret);
    private final ZeroTurretCommand m_zeroTurretCommand = new ZeroTurretCommand(m_turret);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure all the control bindings
        configureControlBindings();
        VisionController.ShooterVision.setControlPoints(Vision.CONTROL_POINTS);
    }

    private void configureControlBindings() {
        
        //Configure control in the same order as the subsystems (alphabetical)

        //Drivetrain
        m_teleopArcadeDriveCommand.setSuppliers(
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickLeftY), JOYSTICK_INPUT_EXPONENT),
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_INPUT_EXPONENT)
        );

        
        // m_flywheelPrototypeTestCommand.setSupplier(
        //     () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightY), JOYSTICK_INPUT_EXPONENT)
        // );

        m_turretManualCommand.setSupplier(
            () -> DriveUtil.powCopySign(joystickOperator.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_INPUT_EXPONENT)
        );

        joystickOperator.getButton(ButtonF310.Y)
        //
        CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopArcadeDriveCommand);
        CommandScheduler.getInstance().setDefaultCommand(m_intake, m_intakeCommand);
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
