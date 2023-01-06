package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveContinuous extends CommandBase {

    private final DrivetrainSubsystem m_drivetrain;
    private final double percentLeft;
    private final double percentRight;
    public DriveContinuous(DrivetrainSubsystem drivetrain, double percentLeft, double percentRight) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        
        this.percentLeft = percentLeft;
        this.percentRight = percentRight;
    }

    @Override
    public void execute() {
        m_drivetrain.driveRaw(percentLeft * SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY, percentRight * SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.driveRaw(0, 0);
    }

}
