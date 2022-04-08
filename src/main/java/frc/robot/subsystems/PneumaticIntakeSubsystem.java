package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;



public class PneumaticIntakeSubsystem extends SubsystemBase {
    private final Compressor m_compressor;
    private final Solenoid m_solenoid;

    public PneumaticIntakeSubsystem() {
        // We don't really need the Compressor object but keeping it for now
        m_compressor = new Compressor(Ports.CAN_PCM, PneumaticsModuleType.CTREPCM);
        m_compressor.enableDigital();
        m_solenoid = new Solenoid(Ports.CAN_PCM, PneumaticsModuleType.CTREPCM, Ports.SOLENOID_PORT);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Extended", m_solenoid.get());
    }

    public void extend()
    {
        m_solenoid.set(true);
    }

    public void retract()
    {
        m_solenoid.set(false);
    }

}
