package frc.robot.commands;

import static frc.robot.Constants.*;

import frc.robot.DriveUtil;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SlewRateArcadeDriveCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrain;
    private DoubleSupplier m_speedSupplier, m_rotationSupplier;
    //Slew rate values to stop tilting
    //Higher = more responsiveness and more tipping
    //Lower = less responsiveness and less tipping
    //If it is to low, it just wont move unless you hold down the joystick for a while
    //Too high = instant response and a lot of tipping
    private SlewRateLimiter leftFilter = new SlewRateLimiter(1600);
    private SlewRateLimiter rightFilter = new SlewRateLimiter(1600);
    public SlewRateArcadeDriveCommand(DrivetrainSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void setSuppliers(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier) {
        m_speedSupplier = speedSupplier;
        m_rotationSupplier = rotationSupplier;
    }

    @Override
    public void execute() {
        double[] speeds = DriveUtil.arcadeToTankDrive(m_speedSupplier.getAsDouble() * ARCADE_SPEED_MULTIPLIER, m_rotationSupplier.getAsDouble() * ARCADE_ROTATION_MULTIPLIER);
        // Convert speeds to target speeds in meters per second, and then divide by hypothetical maximum movement speed
        // Proportion of max speed
        speeds[0] *= SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        speeds[1] *= SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        
        //m_drivetrain.driveRaw(speeds[0], speeds[1]);
        m_drivetrain.driveRaw(leftFilter.calculate(speeds[0]), rightFilter.calculate(speeds[1]));
    }

}
