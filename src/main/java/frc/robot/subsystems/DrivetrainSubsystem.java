package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX m_leftFrontTalon, m_leftRearTalon, m_rightFrontTalon, m_rightRearVictor;
  private final Constants constants = new Constants();
    public DrivetrainSubsystem() {
        m_leftFrontTalon = new WPI_TalonSRX(constants.LEFT_FRONT_TALON_ID);
        m_leftRearTalon = new WPI_TalonSRX(constants.LEFT_REAR_TALON_ID);
        m_rightFrontTalon = new WPI_TalonSRX(constants.RIGHT_FRONT_TALON_ID);
        m_rightRearVictor = new WPI_TalonSRX(constants.RIGHT_REAR_TALON_ID);

        configureMotors();
    }

    @Override
    public void periodic()
    {

    }

    public void drivePO(double left, double right) {
        m_leftFrontTalon.set(ControlMode.PercentOutput, left);
        m_rightFrontTalon.set(ControlMode.PercentOutput, right);
    }

    private void configureMotors() {
        
        //First setup talons with default settings
        m_leftFrontTalon.configFactoryDefault();
        m_leftRearTalon.configFactoryDefault();
        m_rightFrontTalon.configFactoryDefault();
        m_rightRearVictor.configFactoryDefault();


        m_rightFrontTalon.setInverted(true);
        m_rightRearVictor.setInverted(true);
        

        m_leftRearTalon.follow(m_leftFrontTalon, FollowerType.PercentOutput);
        m_rightRearVictor.follow(m_rightFrontTalon, FollowerType.PercentOutput);
    }
}



