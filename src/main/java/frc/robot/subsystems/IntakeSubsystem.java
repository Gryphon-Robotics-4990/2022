package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_intakeLeft, m_intakeRight;

    public IntakeSubsystem() {
        m_intakeLeft = new WPI_TalonSRX(Ports.CAN_INTAKE_LEFT_TALONSRX);
        m_intakeRight = new WPI_TalonSRX(Ports.CAN_INTAKE_RIGHT_TALONSRX);
        configureMotors();
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        m_intakeLeft.set(ControlMode.PercentOutput, leftSpeed);
        m_intakeRight.set(ControlMode.PercentOutput, rightSpeed);
    }

    private void configureMotors() {
        m_intakeLeft.configFactoryDefault();
        m_intakeRight.configFactoryDefault();

        m_intakeLeft.setInverted(false);
        m_intakeRight.setInverted(true);
    }
}