package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeCommand extends CommandBase{
    private final IntakeSubsystem m_intake;
    private double speed = 0.5;

    public ToggleIntakeCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.setSpeed(speed);
        // m_intake.extend();
    }

    @Override
    public void end(boolean interruptible) {
        m_intake.setSpeed(0);
        // m_intake.retract();
    }
}
