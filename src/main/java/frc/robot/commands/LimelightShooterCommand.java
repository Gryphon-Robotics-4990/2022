package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.VisionController;

import static frc.robot.Constants.*;

public class LimelightShooterCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    public LimelightShooterCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double topSpeed = VisionController.ShooterVision.getShooterSpeedFromLimelight();
        double bottomSpeed = 0;
        m_shooter.shootPID(topSpeed, bottomSpeed);
    }
}