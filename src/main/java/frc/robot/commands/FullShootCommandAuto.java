
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;


public class FullShootCommandAuto extends ParallelRaceGroup {
    
    public FullShootCommandAuto(ShooterSubsystem shooter, PreShooterSubsystem preShooter) {
        ShooterPIDCommandAuto shoot = new ShooterPIDCommandAuto(shooter);
        PreShooterCommand preShoot = new PreShooterCommand(preShooter);
        
        addCommands(
            // Shooting
            shoot,
            // Pre-Shooter running 
            new SequentialCommandGroup(
                // Once we PID tune, the isReady() will work
                /*new WaitUntilCommand(() -> shooter.isReady()),*/
                new WaitCommand(2.5),
                new ParallelRaceGroup(new WaitCommand(2), preShoot)
            )
        );
    }
}