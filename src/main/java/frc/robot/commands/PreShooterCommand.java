package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PreShooterSubsystem;

public class PreShooterCommand extends CommandBase{
    private final PreShooterSubsystem m_preShooterSubsystem;
    private final float speed;
    //speed is the speed of the pre-shooter motors, on a scale from 0 to 1 (1 being full power and 0 being no power)

    public void execute() {
        m_preShooterSubsystem.preShootPO(speed);
    }

    public void end(boolean interrupted) {
        //interrupted is a required parameter, but we don't use it for anything
        m_preShooterSubsystem.preShootPO(0);
        //Runs the pre-shooter at 0% power
    }
}
