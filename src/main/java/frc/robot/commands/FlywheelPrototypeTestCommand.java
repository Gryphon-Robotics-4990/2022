package frc.robot.commands;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FlywheelPrototypeTestCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrain;
    private DoubleSupplier m_speedSupplier;

    public FlywheelPrototypeTestCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void setSupplier(DoubleSupplier speed){
        m_speedSupplier = speed;
    }
    @Override
    public void execute() {
        double speed = m_speedSupplier.getAsDouble();
        // Right side has connected motors
        m_drivetrain.drivePO(0, speed);
    }
}
