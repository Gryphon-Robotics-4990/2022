package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticIntakeSubsystem;

public class ToggleIntakeExtensionCommand extends CommandBase {
    private final PneumaticIntakeSubsystem m_intake;

    // Basically a StartEndCommand (we'll change it to this)
    public ToggleIntakeExtensionCommand(PneumaticIntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.extend();
    }

    @Override
    public void end(boolean interruptible) {
        m_intake.retract();
    }
}
