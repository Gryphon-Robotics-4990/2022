package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.DriveUtil;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopArcadeDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrain;
    private DoubleSupplier m_speedSupplier, m_rotationSupplier;

    public TeleopArcadeDriveCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void setSuppliers(DoubleSupplier left, DoubleSupplier right) {
        m_speedSupplier = left;
        m_rotationSupplier = right;
    }

    @Override
    public void execute() {
        double[] speeds = DriveUtil.arcadeToTankDrive(m_speedSupplier.getAsDouble() * ARCADE_SPEED_MULTIPLIER, m_rotationSupplier.getAsDouble() * ARCADE_ROTATION_MULTIPLIER);
        // Convert speeds to target speeds in meters per second, and then divide by hypothetical maximum movement speed
        // Proportion of max speed
        double factor = 0.5/*SubsystemConfig.DRIVETRAIN_MAXIMUM_CRUISE_SPEED_METERS_PER_SECOND / SubsystemConfig.DRIVETRAIN_MAXIMUM_MOVEMENT_SPEED_METERS_PER_SECOND*/;
        speeds[0] *= factor * SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        speeds[1] *= factor * SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        if (speeds[0] != 0 && speeds[1] != 0) System.out.println(speeds[0] + " " + speeds[1]);
        m_drivetrain.driveRaw(speeds[0], speeds[1]);
    }

}
