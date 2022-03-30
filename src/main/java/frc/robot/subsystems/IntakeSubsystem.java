package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;



public class IntakeSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_intakeRight;
    private final DigitalInput m_breakbeam;


    public IntakeSubsystem() {
        m_intakeRight = new WPI_TalonSRX(Ports.CAN_INTAKE_RIGHT_TALONSRX);
        m_breakbeam = new DigitalInput(Ports.DIO_BREAKBEAM);
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

    private void configureMotors() {
        m_intakeRight.configFactoryDefault();

        m_intakeRight.setInverted(false);
    }
}
