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
    //private final Compressor m_compressor;
    //private final Solenoid m_leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.LEFT_SOLENOID_PORT);
    //private final Solenoid m_rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.RIGHT_SOLENOID_PORT);

    public IntakeSubsystem() {
        m_intakeRight = new WPI_TalonSRX(Ports.CAN_INTAKE_RIGHT_TALONSRX);
        m_breakbeam = new DigitalInput(Ports.DIO_BREAKBEAM);
        //m_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        //m_leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.LEFT_SOLENOID_PORT);
        //m_rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.RIGHT_SOLENOID_PORT);

        configureMotors();
    }

    public void setSpeed(double speed) {
        m_intakeRight.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Ball", hasBall());
        SmartDashboard.putBoolean("Intake On", isOn());
    }

    public boolean hasBall() {
        // If beam is broken, ball is there
        return !m_breakbeam.get();
    }

    public boolean isOn() {
        return m_intakeRight.getMotorOutputPercent() != 0;
    }

    public void extend()
    {
        //m_rightSolenoid.set(true);
        //m_leftSolenoid.set(true);
    }

    public void retract()
    {
        //m_rightSolenoid.set(false);
        //m_leftSolenoid.set(false);
    }


    private void configureMotors() {
        m_intakeRight.configFactoryDefault();

        m_intakeRight.setInverted(false);
    }
}
