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



public class IntakeSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_intakeRight;
    private final DigitalInput m_breakbeam;
    private final Compressor m_compressor;
    private final Solenoid m_solenoid;

    public IntakeSubsystem() {
        m_intakeRight = new WPI_TalonSRX(Ports.CAN_INTAKE_RIGHT_TALONSRX);
        m_breakbeam = new DigitalInput(Ports.DIO_BREAKBEAM);
        // We don't really need the Compressor object but keeping it for now
        m_compressor = new Compressor(Ports.CAN_PCM, PneumaticsModuleType.CTREPCM);
        m_compressor.enableDigital();
        m_solenoid = new Solenoid(Ports.CAN_PCM, PneumaticsModuleType.CTREPCM, Ports.SOLENOID_PORT);
        configureMotors();
    }

    public void setSpeed(double speed) {
        m_intakeRight.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Ball", hasBall());
        SmartDashboard.putBoolean("Intake On", isOn());
        SmartDashboard.putBoolean("Intake Extended", isExtended());
        SmartDashboard.putBoolean("Compressor Enabled", m_compressor.enabled());
    }

    public boolean hasBall() {
        // If beam is broken, ball is there
        return !m_breakbeam.get();
    }

    public boolean isOn() {
        return m_intakeRight.getMotorOutputPercent() != 0;
    }

    public boolean isExtended() {
        return m_solenoid.get();
    }

    public void extend()
    {
        m_solenoid.set(true);
    }

    public void retract()
    {
        m_solenoid.set(false);
    }

    public void stopCompressor()
    {
        m_compressor.disable();
    }

    public void startAnalogCompressor(){
        m_compressor.enableDigital();
    }


    private void configureMotors() {
        m_intakeRight.configFactoryDefault();

        m_intakeRight.setInverted(false);
    }
}
