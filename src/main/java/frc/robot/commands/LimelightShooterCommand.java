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
        //double[] speeds = VisionController.ShooterVision.getShooterSpeedFromLimelight();
        double[] speeds = {0.0, 0.0};
        double topSpeed = speeds[0];
        double bottomSpeed = speeds[1];
        m_shooter.shootPID(topSpeed, bottomSpeed);
        SmartDashboard.putBoolean("Shooter Ready", m_shooter.isReady());
        SmartDashboard.putNumber("Shooter Velocity", m_shooter.getVelocityBottom());
    }
}