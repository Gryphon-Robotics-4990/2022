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

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

public class AutoCommand extends ParallelCommandGroup {

    private final DrivetrainSubsystem m_drivetrain;

    // Constructor
    public AutoCommand(DrivetrainSubsystem drive, PreShooterSubsystem preShooter, ShooterSubsystem shooter, TurretSubsystem turret) {
        m_drivetrain = drive;

        int preShooterRunLength = 2;
        // Distance to move backwards (meters)
        double distanceBack = -0.75;
        ParallelRaceGroup driveCommand = getDriveBackCommand(distanceBack);


        addCommands(
            new ShooterPIDCommand(shooter),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    driveCommand,
                    new SequentialCommandGroup(
                        new ParallelRaceGroup(
                            new TurretPositionCommand(turret),
                            new WaitCommand(2)
                        ),
                        new ParallelRaceGroup(
                            //new WaitUntilCommand(() -> turret.isReady()),
                            new WaitCommand(2),
                            //new RunCommand(() -> turret.setPositionDegrees(-90), turret)
                            new LimelightTurretAimCommand(turret)
                        )
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
        double percentSpeed = -0.5;
        double linearSpeed = percentSpeed * SubsystemConfig.DRIVETRAIN_MAXIMUM_MOVEMENT_SPEED_METERS_PER_SECOND;
        //divide by (maxSpeed/2), which is 2.015 meters per second, so we get the duration we need
        double duration = meters / linearSpeed;
        System.out.println(duration);
        ParallelRaceGroup driveCommand = new ParallelRaceGroup(
            new WaitCommand(duration), 
            new DriveContinuous(m_drivetrain, percentSpeed, percentSpeed)
        );
        return driveCommand;
    }
}