package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretManualCommand extends CommandBase{
    private final TurretSubsystem m_turret;
    private DoubleSupplier m_speedSupplier;

    public TurretManualCommand(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(turret);
    }


    public void setSupplier(DoubleSupplier supplier) {
        m_speedSupplier = supplier;
    }

    //set motor speeds
    @Override
    public void execute() {
        double driveSpeed = m_speedSupplier.getAsDouble();
        m_turret.drivePO(driveSpeed);
        //System.out.println(Limelight.getCrosshairHorizontalOffset());
        //System.out.println(Limelight.getCrosshairVerticalOffset());
    }

}
