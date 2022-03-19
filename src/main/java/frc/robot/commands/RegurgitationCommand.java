package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;


public class RegurgitationCommand extends CommandBase{
    private final IntakeSubsystem m_intake;
    private final PreShooterSubsystem m_preShooter;

    public RegurgitationCommand(IntakeSubsystem intake, PreShooterSubsystem preShooter)
    {
        m_intake = intake;
        m_preShooter = preShooter;

        addRequirements(intake, preShooter);
    }

    public void execute()
    {
        double speed = -0.6;
        m_intake.setSpeed(speed);
        m_preShooter.setSpeed(speed);
    }

    @Override
    public void end(boolean interruptible) {
        m_preShooter.setSpeed(0);
        m_intake.setSpeed(0);
    }
}
