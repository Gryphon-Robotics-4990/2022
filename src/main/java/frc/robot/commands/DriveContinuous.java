package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveContinuous extends CommandBase {

    private final DrivetrainSubsystem m_drivetrain;
    private final double speedLeft;
    private final double speedRight;
    public DriveContinuous(DrivetrainSubsystem drivetrain, double speedLeft, double speedRight) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.speedLeft = speedLeft;
        this.speedRight = speedRight;
    }

    @Override
    public void execute() {
        m_drivetrain.driveRaw(speedLeft * SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY, speedRight * SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.driveRaw(0, 0);
    }

}
