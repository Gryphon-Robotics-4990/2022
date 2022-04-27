package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PreShooterSubsystem;

public class PreShooterCommand extends CommandBase{
    private final PreShooterSubsystem m_preShooter;
    private double speed = 0.8;

    // Constructor
    public PreShooterCommand(PreShooterSubsystem preShooter) {
        m_preShooter = preShooter;
        addRequirements(preShooter);
    }

    //Execute - call the subsystem
    @Override
    public void execute() {
        m_preShooter.setSpeed(speed);
    }
    //End - stop the motors
    @Override
    public void end(boolean interruptible) {
        m_preShooter.setSpeed(0);
    }
}
