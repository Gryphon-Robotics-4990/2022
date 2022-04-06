package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.TurretSubsystem;

// NOTE: this may not be necessary, look at the current solution with the default command
public class SwitchTurretModesCommand extends CommandBase {
    private final TurretSubsystem m_turret;

    public SwitchTurretModesCommand(TurretSubsystem turret, 
    LimelightTurretAimCommand autoTurret, 
    TurretManualCommand manualTurret, ZeroTurretCommand zeroTurret) {
        m_turret = turret;
        // No need to add requirements, because the commands called require the subsystem
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {

    }
}
