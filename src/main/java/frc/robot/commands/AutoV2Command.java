package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import static frc.robot.Constants.*;


public class AutoV2Command extends SequentialCommandGroup {

    private final DrivetrainSubsystem m_drivetrain;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;
    private final PreShooterSubsystem m_preShooter;
    // Constructor
    public AutoV2Command(DrivetrainSubsystem drive, PreShooterSubsystem preShooter, ShooterSubsystem shooter, TurretSubsystem turret, IntakeSubsystem intake) {
        m_drivetrain = drive;
        m_intake = intake;
        m_shooter = shooter;
        m_preShooter = preShooter;
        // Distance to move forwards (meters)
        double distanceBall = 2;
        double distanceReturn = -distanceBall;
        double percentSpeed = 0.2;
        ParallelRaceGroup driveCommand = getDriveBackCommand(distanceBall, percentSpeed);
        ParallelRaceGroup driveReturnCommand = getDriveBackCommand(distanceReturn, -percentSpeed);

        addCommands(
            new ParallelRaceGroup(
                driveCommand,
                new ParallelCommandGroup(
                    new PreShooterBurstCommand(m_preShooter),
                    new ToggleIntakeCommand(m_intake)
                )
            ),
            driveReturnCommand,
            new ParallelRaceGroup(
                //Change the parameter of this waitCommand to the number of seconds to turn for:
                new WaitCommand(0.75),
                new DriveContinuous(m_drivetrain, percentSpeed, -percentSpeed)
            ),
            //new ParallelRaceGroup(
            //     new WaitUntilCommand(() -> turret.isReady()),
            //     new WaitCommand(2),
            //     new LimelightTurretAimCommand(turret)
            //)
            new FullShootCommandAuto(m_shooter, m_preShooter)
        );
    }

    public ParallelRaceGroup getDriveBackCommand(double meters, double speed){
        double percentSpeed = speed;
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
