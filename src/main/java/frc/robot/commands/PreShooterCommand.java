package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PreShooterSubsystem;

public class PreShooterCommand extends CommandBase{
    private final PreShooterSubsystem m_preShooterSubsystem;

    public PreShooterCommand(PreShooterSubsystem preShooter) {
        m_preShooterSubsystem = preShooter;
        addRequirements(preShooter);
    }

    public void execute() {
        //speed of the pre-shooter motors, on a scale from 0 to 1 (1 being full power and 0 being no power)
        double speed = 0.5;
        m_preShooterSubsystem.setSpeed(speed);
    }

    public void end(boolean interrupted) {
        //interrupted is a required parameter, but we don't use it for anything
        //Runs the pre-shooter at 0% power
        m_preShooterSubsystem.setSpeed(0);
    }
}
