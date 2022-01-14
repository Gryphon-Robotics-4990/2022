package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.DriveUtil;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDTestDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrain;

    public PIDTestDriveCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double speed = 0.2;
        speed *= SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        m_drivetrain.driveRaw(speed, speed);
    }

    @Override
    public void end(boolean interrupted) {
        double speed = 0;
        speed *= SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        m_drivetrain.driveRaw(speed, speed);
    }

}
