package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem m_intake;
    
    public IntakeCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    //set motor speeds
    @Override
    public void execute() {
        double speed = 0.5;
        m_intake.setSpeed(speed);
    }
}
