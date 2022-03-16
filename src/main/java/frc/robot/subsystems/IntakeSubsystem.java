package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_intakeRight;
    //private final DigitalInput m_breakbeam;

    public IntakeSubsystem() {
        m_intakeRight = new WPI_TalonSRX(Ports.CAN_INTAKE_RIGHT_TALONSRX);
        //m_breakbeam = new DigitalInput(Ports.DIO_BREAKBEAM);
        configureMotors();
    }

    public void setSpeed(double speed) {
        m_intakeRight.set(ControlMode.PercentOutput, speed);
    }


    // @Log
    // public boolean hasBall() {
    //     return m_breakbeam.get();
    // }

    private void configureMotors() {
        m_intakeRight.configFactoryDefault();

        m_intakeRight.setInverted(false);
    }
}
