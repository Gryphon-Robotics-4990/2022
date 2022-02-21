package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

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
        double speed = 0;
        m_turret.drivePO(m_speedSupplier.getAsDouble());
    }

}
