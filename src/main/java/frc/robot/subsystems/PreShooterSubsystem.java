package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class PreShooterSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_preShooter;

    public PreShooterSubsystem() {
        m_preShooter = new WPI_TalonSRX(Ports.CAN_PRESHOOTER_TALONSRX);
        configureMotors();
    }

    public void setSpeed(double speed) {
        m_preShooter.set(ControlMode.PercentOutput, speed);
    }

    private void configureMotors() {
        m_preShooter.configFactoryDefault();

        m_preShooter.setInverted(false);
    }
}
