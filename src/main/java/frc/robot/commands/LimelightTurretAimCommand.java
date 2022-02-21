package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.VisionController;
import frc.robot.subsystems.TurretSubsystem;

public class LimelightTurretAimCommand extends CommandBase {

    private final TurretSubsystem m_turret;

    public LimelightTurretAimCommand(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // TODO find necessary turret position using VisionController
        double current = m_turret.getEncoderPosition();
        double change = VisionController.TurretVision.getTurretPositionFromAngle();
        m_turret.setPosition(current + change);
    }
    
}
