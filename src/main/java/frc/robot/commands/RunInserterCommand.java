package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Direction;
import frc.robot.subsystems.InserterSubsystem;

import static frc.robot.Constants.*;

public class RunInserterCommand extends CommandBase {

    private final InserterSubsystem m_inserter;
    private final Direction m_direction;


    public RunInserterCommand(InserterSubsystem inserter, Direction direction) {
        addRequirements(inserter);
        m_inserter = inserter;

        m_direction = direction;
    }

    @Override
    public void execute() {
        m_inserter.run(m_direction);
    }

    @Override
    public void end(boolean interrupted) {
        m_inserter.run(Direction.Stop);
    }
}