package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Units;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPositionCommand extends CommandBase{
    private final TurretSubsystem m_turret;
    private final double position = -90 * Units.DEGREE.to(Units.ENCODER_ANGLE) * 14;

    public TurretPositionCommand(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(turret);
        
    }

    //set motor speeds
    @Override
    public void execute() {
        m_turret.setPosition(m_turret.getEncoderPosition() + position);
    }

}
