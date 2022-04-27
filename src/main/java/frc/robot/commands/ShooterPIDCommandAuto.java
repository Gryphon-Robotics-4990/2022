
package frc.robot.commands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDCommandAuto extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    // Once we PID tune
    // Upper Hub
    private final double bottomSpeed = 28000;//28000 is good, 30000 maybe too high 25000 set by Declan on load in day
    public ShooterPIDCommandAuto(ShooterSubsystem shooter) {
        m_shooterSubsystem = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        // Once we PID tune
        m_shooterSubsystem.shootPID(bottomSpeed);
        //m_shooterSubsystem.shootPO(0, bottomPercent);

    }
    @Override
    public void end(boolean interrupted) {
        // Once we PID tune
        //m_shooterSubsystem.shootPID(0);
        // When we set PID to 0, it spikes
        m_shooterSubsystem.shootPO(0);
    }
}