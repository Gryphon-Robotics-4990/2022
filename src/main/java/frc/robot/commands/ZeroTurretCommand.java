package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ZeroTurretCommand extends CommandBase{
    private final TurretSubsystem m_turret;

    public ZeroTurretCommand(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(turret);
    }
    @Override
    public void execute() {
        m_turret.driveRaw(0);
    }
}