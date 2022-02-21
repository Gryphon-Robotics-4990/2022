package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualAutomaticToggleCommand extends CommandBase{
    private final TurretSubsystem m_turret;

    public ManualAutomaticToggleCommand(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute {
        joystickOperator.getButton(ButtonF310.Y).toggleWhenPressed(new StartEndCommand(TurretSubsystem::TurretManualCommand, TurretSubsystem::LimelightTurretAimCommand, TurretSubsystem));
    }
    @Override
    public void end(boolean interrupted) {
        
    }
}
