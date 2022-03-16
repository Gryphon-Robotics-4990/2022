package frc.robot.commands;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.BreakbeamSubsystem;
import io.github.oblarg.oblog.annotations.Log;

public class BreakbeamCommand extends CommandBase{
    private final BreakbeamSubsystem m_breakBeam;
    
    public BreakbeamCommand(BreakbeamSubsystem breakbeam) {
        m_breakBeam = breakbeam;
    }

    //set motor speeds
    @Override
    public void execute() {
        SmartDashboard.putBoolean("Has Ball", m_breakBeam.hasBall());
    }
}