package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotMeasurements;
import frc.robot.Constants.Units;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoMoveShootBallCommand extends ParallelCommandGroup {


    public AutoMoveShootBallCommand(DrivetrainSubsystem drivetrain, ShooterSubsystem shooterSubsystem, 
        ShooterPIDCommand shooterCommand, PreShooterCommand preShooterCommand) {
        
        DriveBackwardsCommand driveBack = new DriveBackwardsCommand(drivetrain);
        WaitToShootCommand waitToShoot = new WaitToShootCommand(drivetrain, shooterSubsystem);

        addCommands(
            driveBack,
            shooterCommand,
            // We need to wait for the shooter to be ready
            // before we send the ball up
            new SequentialCommandGroup(
                waitToShoot,
                preShooterCommand
            )
        );
    }

    private class DriveBackwardsCommand extends CommandBase {
        
        DrivetrainSubsystem m_drivetrain;
        double encoder_pos;
        
        public DriveBackwardsCommand(DrivetrainSubsystem drive) {
            m_drivetrain = drive;
            addRequirements(drive);

            // How much we're moving back by (in feet)
            double back_by = 7.5;
            // Arc length (inches) -> Radians
            double ang_pos = (back_by * 12 / RobotMeasurements.DRIVETRAIN_WHEEL_RADIUS);
            // Radians -> Encoder ticks
            encoder_pos = ang_pos * Units.RADIAN.to(Units.ENCODER_ANGLE);
            
        }

        @Override
        public void execute() {
            m_drivetrain.driveToPosition(encoder_pos);
        }
    }

    private class WaitToShootCommand extends CommandBase {
        
        DrivetrainSubsystem m_drivetrain;
        ShooterSubsystem m_shoot;

        public WaitToShootCommand(DrivetrainSubsystem drive, ShooterSubsystem shoot) {
            m_drivetrain = drive;
            m_shoot = shoot;
        }

        @Override
        public boolean isFinished() {
            return m_drivetrain.isInPosition() && m_shoot.isReady();
        }
    }
}
