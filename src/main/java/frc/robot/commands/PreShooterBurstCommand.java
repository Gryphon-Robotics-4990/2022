package frc.robot.commands;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.PreShooterSubsystem;

public class PreShooterBurstCommand extends ParallelRaceGroup{
    
    public PreShooterBurstCommand(PreShooterSubsystem preShooter) {
        addCommands(
            new PreShooterCommand(preShooter),
            new WaitCommand(1.5)
        );       
    }
}
