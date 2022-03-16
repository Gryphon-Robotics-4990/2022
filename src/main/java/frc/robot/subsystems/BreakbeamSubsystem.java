package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;

public class BreakbeamSubsystem extends SubsystemBase {

    //private final DigitalInput m_breakbeam;

    public BreakbeamSubsystem() {
        //m_breakbeam = new DigitalInput(Ports.DIO_BREAKBEAM);
    }

    @Log
    public boolean hasBall() {
        //return m_breakbeam.get();
        return true;
    }
}
