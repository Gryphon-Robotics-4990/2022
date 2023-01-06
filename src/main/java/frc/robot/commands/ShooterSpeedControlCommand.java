package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.DriveUtil;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterSpeedControlCommand extends CommandBase {

    private final ShooterSubsystem m_shooter;
    private DoubleSupplier m_speedVelocity;

    public ShooterSpeedControlCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    public void setSuppliers(DoubleSupplier speedSupplier) {
        m_speedVelocity = speedSupplier;
    }

    @Override
    public void execute() {

        m_shooter.shootPO(m_speedVelocity.getAsDouble() * 0.6);
    }

}
