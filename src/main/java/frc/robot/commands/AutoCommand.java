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


public class AutoCommand extends ParallelCommandGroup {

    private final DrivetrainSubsystem m_drivetrain;

    // Constructor
    public AutoCommand(DrivetrainSubsystem drive, PreShooterSubsystem preShooter, ShooterSubsystem shooter, TurretSubsystem turret) {
        m_drivetrain = drive;

        int preShooterRunLength = 2;
        // Distance to move backwards (meters)
        double distanceBack = -1.5;
        ParallelRaceGroup driveCommand = getDriveBackCommand(distanceBack);


        addCommands(
            // Keep the shooter running for first 4 sec of auto
            // TODO stop shooter after ball shoots
            new ParallelRaceGroup(
                new ShooterPIDCommand(shooter),
                new WaitCommand(4)
            ),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    driveCommand,
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
                new ParallelRaceGroup(
                    new PreShooterCommand(preShooter), 
                    new WaitCommand(preShooterRunLength)
                )
            )
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
