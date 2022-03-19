package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotMeasurements;
import frc.robot.Constants.Units;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import static frc.robot.Constants.*;

public class AutoMoveShootBallCommand extends ParallelCommandGroup {

    private final DrivetrainSubsystem m_drivetrain;

    public AutoMoveShootBallCommand(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, 
        TurretSubsystem turret, PreShooterSubsystem preShooter) {
        m_drivetrain = drivetrain;
        
        Command driveBack = getDriveBackCommand();
        //WaitToShootCommand waitToShoot = new WaitToShootCommand(drivetrain, shooter);
        //TurnTurretCommand turnTurretCommand = new TurnTurretCommand(turret);
        //LimelightTurretAimCommand limelightTurretCommand = new LimelightTurretAimCommand(turret);
        //ShooterPIDCommand shooterCommand = new ShooterPIDCommand(shooter);
        //PreShooterCommand preShooterCommand = new PreShooterCommand(preShooter);

        addCommands(
            driveBack
            //shooterCommand
            //new SequentialCommandGroup(turnTurretCommand, limelightTurretCommand)
            // // We need to wait for the shooter to be ready
            // // before we send the ball up
            // new SequentialCommandGroup(waitToShoot, preShooterCommand)
        );
    }

    private ParallelDeadlineGroup getDriveBackCommand() {
        // How much we're moving back by (in feet)
        double back_by = -4;
        double back_by_meters = back_by * Units.FEET.to(Units.METER);

        double percentSpeed = -0.5;

        double encoderVelocity = percentSpeed * SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        double velocityMetersSeconds = encoderVelocity * Units.ENCODER_ANGULAR_VELOCITY.to(Units.METERS_PER_SECOND);
        System.out.println(encoderVelocity);
        System.out.println(velocityMetersSeconds);
        // How much time we need to move back for to get the desired distance
        double time = back_by_meters / velocityMetersSeconds;

        System.out.println(time);
        time = 1.0;
        Command wait = new WaitCommand(time);
        Command driveCommand = new RunCommand(() -> m_drivetrain.driveRaw(encoderVelocity, encoderVelocity), m_drivetrain);
        
        return new ParallelDeadlineGroup(wait, wait, driveCommand);

        // Old code (requires position PID to work)
        // // Arc length (inches) -> Radians
        // //double ang_pos = (back_by * 12 / RobotMeasurements.DRIVETRAIN_WHEEL_RADIUS);
        // double ang_pos = -4096.0;
        // // Radians -> Encoder ticks
        // encoder_pos = ang_pos * Units.RADIAN.to(Units.ENCODER_ANGLE);
    }

    private class TurnTurretCommand extends InstantCommand {
        int position = -100;
        public TurnTurretCommand(TurretSubsystem turret) {
           turret.setPosition(turret.getEncoderPosition() + position);
        }
    }

    private class WaitToShootCommand extends CommandBase {
        
        private final DrivetrainSubsystem m_drivetrain;
        private final ShooterSubsystem m_shoot;

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
