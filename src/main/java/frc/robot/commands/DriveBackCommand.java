package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveBackCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrain;

    public DriveBackCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.driveBack(1.5);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.driveRaw(0, 0);
    }

}
