
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double topSpeed = 0;
    private final double bottomSpeed = 28000;
    
    public ShooterPIDCommand(ShooterSubsystem shooter) {
        m_shooterSubsystem = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        m_shooterSubsystem.shootPID(topSpeed, bottomSpeed);

    }
    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.shootPID(0,0);
    }
}