package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import static frc.robot.Constants.*;


public class AutoCommand extends SequentialCommandGroup {

    private final DrivetrainSubsystem m_drivetrain;

    // Constructor
    public AutoCommand(DrivetrainSubsystem drive, PreShooterSubsystem preShooter, ShooterSubsystem shooter, TurretSubsystem turret) {
        m_drivetrain = drive;

        // Distance to move backwards (meters)
        double distanceBack = -0.75;
        ParallelRaceGroup driveCommand = getDriveBackCommand(distanceBack);
        ParallelRaceGroup driveTaxiCommand = getDriveBackCommand(-2);
        // 1. Drive backwards and adjust turret together
        // 2. Shoot ball (spins up shooter and then runs pre shooter)
        addCommands(
            new ParallelCommandGroup(
                // delay before we start, in case another team needs it
                new SequentialCommandGroup(
                    new WaitCommand(2),
                    driveCommand
                ),               
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new TurretPositionCommand(turret),
                        new WaitCommand(1)
                    )
                    // FOR LIMELIGHT:
                    // new ParallelRaceGroup(
                    //     //new WaitUntilCommand(() -> turret.isReady()),
                    //     new WaitCommand(2),
                    //     new LimelightTurretAimCommand(turret)
                    // )
                )
            ),
            new FullShootCommandAuto(shooter, preShooter),
            driveTaxiCommand
        );
    }

    public ParallelRaceGroup getDriveBackCommand(double meters){
        double percentSpeed = -0.2;
        double linearSpeed = percentSpeed * SubsystemConfig.DRIVETRAIN_MAXIMUM_CRUISE_SPEED_METERS_PER_SECOND;
        System.out.println(SubsystemConfig.DRIVETRAIN_MAXIMUM_MOVEMENT_SPEED_METERS_PER_SECOND);
        System.out.println(linearSpeed);
        // divide distance by speed, so we get the duration we need
        double duration = meters / linearSpeed;
        System.out.println("Drive back time: " + Double.toString(duration));
        ParallelRaceGroup driveCommand = new ParallelRaceGroup(
            new WaitCommand(duration), 
            new DriveContinuous(m_drivetrain, percentSpeed, percentSpeed)
        );
        return driveCommand;
    }
}
