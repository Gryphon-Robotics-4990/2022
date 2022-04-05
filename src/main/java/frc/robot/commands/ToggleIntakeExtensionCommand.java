package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeExtensionCommand extends CommandBase{
    private final IntakeSubsystem m_intake;

    public ToggleIntakeExtensionCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.extend();
    }

    @Override
    public void end(boolean interruptible) {
        m_intake.retract();
    }
}
