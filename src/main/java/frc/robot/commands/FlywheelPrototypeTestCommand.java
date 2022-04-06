package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FlywheelPrototypeTestCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrain;
    // Set to 0.5 to test pre-shooter
    private double bottomSpeed = 0.5;
    private double topSpeed = 0.5;

    public FlywheelPrototypeTestCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.drivePO(topSpeed, bottomSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        double stop = 0;
        m_drivetrain.drivePO(stop, stop);
    }
}
