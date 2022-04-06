package frc.robot.commands;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeChompLoopCommand extends CommandBase{
    private final IntakeSubsystem m_intake;
    boolean temp = true;
    public IntakeChompLoopCommand(IntakeSubsystem intake){
        m_intake = intake;
    }

    @Override
    public void execute()
    {
        while (temp == true)
        {
            m_intake.extend();
            new WaitCommand(2.5);
            m_intake.retract();
        }
        
    }

    @Override 
    public void end(boolean interruptable)
    {
        m_intake.retract();
    }
}
