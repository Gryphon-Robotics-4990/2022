package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SubsystemConfig;
import frc.robot.Constants.Units;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPositionCommand extends CommandBase {
    private final TurretSubsystem m_turret;
    // Counter-clockwise is positive, like angles on unit circle
    private final double position = 90;

    public TurretPositionCommand(TurretSubsystem turret) {
        m_turret = turret;
        addRequirements(turret);

    }

    //set motor speeds
    @Override
    public void execute() {
        m_turret.setPositionDegrees(position);
    }

    @Override
    public boolean isFinished() {
        return m_turret.isReady();
    }

    @Override
    public void end(boolean interrupted) {
        //m_turret.setPositionDegrees(0);
        System.out.println("TurretPositionCommand ended");
    }

}
