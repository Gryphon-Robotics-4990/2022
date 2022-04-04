package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.vision.VisionController;
import frc.robot.JoystickF310.*;

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
    private final PreShooterSubsystem m_preShooter = new PreShooterSubsystem();

    //Create Commands
    private final TeleopArcadeDriveCommand m_teleopArcadeDriveCommand = new TeleopArcadeDriveCommand(m_drivetrain);
    //private final FlywheelPrototypeTestCommand m_flywheelPrototypeTestCommand = new FlywheelPrototypeTestCommand(m_drivetrain);
    //private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intake);
    private final LimelightTurretAimCommand m_limelightTurretAimCommand = new LimelightTurretAimCommand(m_turret);
    private final TurretManualCommand m_turretManualCommand = new TurretManualCommand(m_turret);
    private final ZeroTurretCommand m_zeroTurretCommand = new ZeroTurretCommand(m_turret);
    private final LimelightShooterCommand m_limelightShooterCommand = new LimelightShooterCommand(m_shooter);
    //private final ShooterPOTestCommand m_bottomShooterJoystickTest = new ShooterPOTestCommand(m_shooter);
    private final PreShooterCommand m_preShooterCommand = new PreShooterCommand(m_preShooter);
    private final ToggleIntakeCommand m_toggleIntakeCommand = new ToggleIntakeCommand(m_intake);
    private final RegurgitationCommand m_regurgitationCommand = new RegurgitationCommand(m_intake, m_preShooter);
    private final ShooterPIDCommand m_shooterPIDCommand = new ShooterPIDCommand(m_shooter);
    private final AutoCommand m_autoCommand = new AutoCommand(m_drivetrain, m_preShooter, m_shooter, m_turret);
    private final SlewRateArcadeDriveCommand m_normalDrive = new SlewRateArcadeDriveCommand(m_drivetrain);

    //private final AutoMoveShootBallCommand m_autoMoveShootBallCommand = new AutoMoveShootBallCommand(m_drivetrain, m_shooter,  m_turret, m_preShooter);
    private final FullShootCommand m_fullShoot = new FullShootCommand(m_shooter, m_preShooter);
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure all the control bindings
        configureControlBindings();
        //VisionController.ShooterVision.setControlPoints(Vision.CONTROL_POINTS);
    }

    private void configureControlBindings() {
        
        //Configure control in the same order as the subsystems (alphabetical)

        //Drivetrain
        // m_teleopArcadeDriveCommand.setSuppliers(
        //     () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickLeftY), JOYSTICK_THROTTLE_EXPONENT),
        //     () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_TURNING_EXPONENT)
        // );

        m_normalDrive.setSuppliers(
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickLeftY), JOYSTICK_THROTTLE_EXPONENT),
            () -> DriveUtil.powCopySign(joystickDrive.getRawAxis(AxisF310.JoystickRightX), JOYSTICK_TURNING_EXPONENT)

        );


        m_turretManualCommand.setSupplier(
            () -> DriveUtil.powCopySign(joystickOperator.getRawAxis(AxisF310.JoystickLeftX), JOYSTICK_OPERATOR_EXPONENT)
        );

        joystickOperator.getButton(ButtonF310.Y).toggleWhenPressed(m_turretManualCommand);

        //joystickOperator.getButton(ButtonF310.A).toggleWhenPressed(m_preShooterCommand);

        joystickOperator.getButton(ButtonF310.BumperRight).toggleWhenPressed(m_shooterPIDCommand);
        joystickOperator.getButton(ButtonF310.B).toggleWhenPressed(m_toggleIntakeCommand);

        joystickOperator.getButton(ButtonF310.BumperLeft).toggleWhenPressed(m_regurgitationCommand);
        joystickOperator.getButton(ButtonF310.A).whenPressed(m_fullShoot);
        // Configures the switching between manual and automatic turret modes
        // It's only possible to zero the turret when its button is pressed and the mode is toggled to manual control
        // The turret manual command is automatically scheduled when manual mode is toggled
        // When it's toggled off, the default limelight turret aiming command will run
        
        //TODO find button to zero turret
        // joystickOperator.getButton(ButtonF310.Y).toggleWhenActive(m_turretManualCommand)
        //     .and(joystickOperator.getButton(ButtonF310.B)).toggleWhenActive(m_zeroTurretCommand);
        
        //CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopArcadeDriveCommand);
        CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_normalDrive);
        CommandScheduler.getInstance().setDefaultCommand(m_turret, m_limelightTurretAimCommand);
        //CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shooterPIDCommand);
    }

    public void setTeleopDefaultCommands() {
        //Put default command setters here once auto works
    }

    

    public Command getAutonomousCommand() {
        // Moving backwards and shooting the ball we start with
        //Preshooter now starts teleop phase enabled.
        return m_autoCommand;
    }
}
