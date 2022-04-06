package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;

// This is just an instant command but we gave it its own file
public class ZeroTurretCommand extends InstantCommand {

    public ZeroTurretCommand(TurretSubsystem turret) {
        super(() -> turret.setPosition(0), turret);
    }

}