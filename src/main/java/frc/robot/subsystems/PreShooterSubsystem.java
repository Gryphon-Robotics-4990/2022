package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;


public class PreShooterSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_talon;

    public PreShooterSubsystem() {
        m_talon = new WPI_TalonSRX(Ports.CAN_PRESHOOTER_TALONSRX);
        
        configureMotors();
    }

    public void preShootPO(double percentSpeed) {
        m_talon.set(ControlMode.PercentOutput, percentSpeed);
    }

    private void configureMotors() {
        
        //First setup the talon with default settings
        m_talon.configFactoryDefault();

        m_talon.setInverted(false);

        //Brake mode so no coasting
        m_talon.setNeutralMode(NeutralMode.Brake);
    }
    
}
