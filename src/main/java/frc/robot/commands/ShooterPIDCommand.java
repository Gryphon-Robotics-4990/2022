
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double topSpeed = 0;
    // Once we PID tune
    //private final double bottomSpeed = 26000;//28000 is good, 30000 maybe too high
    private final double bottomPercent = 0.4;

    public ShooterPIDCommand(ShooterSubsystem shooter) {
        m_shooterSubsystem = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        // Once we PID tune
        //m_shooterSubsystem.shootPID(topSpeed, bottomSpeed);
        m_shooterSubsystem.shootPO(0, bottomPercent);

    }
    @Override
    public void end(boolean interrupted) {
        // Once we PID tune
        //m_shooterSubsystem.shootPID(0,0);
        m_shooterSubsystem.shootPO(0,0);
    }
}