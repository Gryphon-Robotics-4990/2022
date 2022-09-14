package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX m_leftFrontTalon, m_leftRearTalon, m_rightFrontTalon, m_rightRearTalon;
  private final Constants constants = new Constants();
    public DrivetrainSubsystem() {
        m_leftFrontTalon = new WPI_TalonSRX(constants.LEFT_FRONT_TALON_ID);
        m_leftRearTalon = new WPI_TalonSRX(constants.LEFT_REAR_TALON_ID);
        m_rightFrontTalon = new WPI_TalonSRX(constants.RIGHT_FRONT_TALON_ID);
        m_rightRearTalon = new WPI_TalonSRX(constants.RIGHT_REAR_TALON_ID);

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

    public void driveVelocity(double left, double right) {
        m_leftFrontTalon.set(ControlMode.Velocity, left);
        m_rightFrontTalon.set(ControlMode.Velocity, right);
    }

    private void configureMotors() {
        
        //First setup talons with default settings
        m_leftFrontTalon.configFactoryDefault();
        m_leftRearTalon.configFactoryDefault();
        m_rightFrontTalon.configFactoryDefault();
        m_rightRearTalon.configFactoryDefault();
        
        m_leftFrontTalon.setSensorPhase(true);
        m_rightFrontTalon.setSensorPhase(true);

        m_rightFrontTalon.setInverted(true);
        m_rightRearTalon.setInverted(true);

        m_leftRearTalon.follow(m_leftFrontTalon, FollowerType.PercentOutput);
        m_rightRearTalon.follow(m_rightFrontTalon, FollowerType.PercentOutput);

        m_leftFrontTalon.configSelectedFeedbackSensor(Constants.TALON_DEFAULT_FEEDBACK_DEVICE, Constants.TALON_DEFAULT_PID_ID, 5);
        m_rightFrontTalon.configSelectedFeedbackSensor(Constants.TALON_DEFAULT_FEEDBACK_DEVICE, Constants.TALON_DEFAULT_PID_ID, 5);
        
        TalonSRXConfiguration cLeft = new TalonSRXConfiguration();
        TalonSRXConfiguration cRight = new TalonSRXConfiguration();

        NeutralMode mode = NeutralMode.Coast;
        m_leftFrontTalon.setNeutralMode(mode);
        m_rightFrontTalon.setNeutralMode(mode);
        m_leftRearTalon.setNeutralMode(mode);
        m_rightRearTalon.setNeutralMode(mode);

        m_leftFrontTalon.configAllSettings(cLeft);
        m_rightFrontTalon.configAllSettings(cRight);
        
    }
}



