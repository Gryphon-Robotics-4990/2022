package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPOTestCommand extends CommandBase{
    private final ShooterSubsystem m_shooter;

    private double speed = 0.5;
    private DoubleSupplier m_supplier;

    public ShooterPOTestCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    public void setSuppliers(DoubleSupplier supplier)
    {
        m_supplier = supplier;
    }

    @Override
    public void execute() {
        m_shooter.shootPO(0, m_supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        double stop = 0;
        m_shooter.shootPO(stop, stop);
    }
}
