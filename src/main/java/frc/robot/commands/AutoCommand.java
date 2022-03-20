package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import jdk.nashorn.api.tree.NewTree;

public class AutoCommand extends ParallelCommandGroup {
    // Constructor
    public AutoCommand(PreShooterSubsystem preShooter, ShooterSubsystem shooter, DrivetrainSubsystem drivetrain, TurretSubsystem turret) {

        int preShooterRunLength = 2;
        addCommands(
            new ShooterPIDCommand(shooter),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new DriveBackCommand(drivetrain),
                    new SequentialCommandGroup(
                        new TurretPositionCommand(turret),
                        new ParallelRaceGroup(
                            new WaitUntilCommand(() -> turret.isReady()),
                            new LimelightTurretAimCommand(turret)
                        )
                    )
                ),
                new ParallelRaceGroup(
                    new PreShooterCommand(preShooter), 
                    new WaitCommand(preShooterRunLength)
                ))
        );
    }
}
