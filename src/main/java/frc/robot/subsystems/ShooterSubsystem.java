package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_rightBottomTalon, m_leftBottomTalon;

    public ShooterSubsystem() {
        //The top and right bottom talons use their own PID, but the bottom left just follows the bottom right.
        m_rightBottomTalon = new WPI_TalonSRX(Ports.CAN_SHOOTER_RIGHT_BOTTOM_TALONSRX);
        // Left shooter had wiring issue
        m_leftBottomTalon = new WPI_TalonSRX(Ports.CAN_SHOOTER_LEFT_BOTTOM_TALONSRX);

        configureMotors();
        m_rightBottomTalon.setSelectedSensorPosition(0);
        m_rightBottomTalon.configPeakCurrentLimit(30);
        m_leftBottomTalon.configPeakCurrentLimit(30);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Shooter On", isOn());
    }

    public void shootPID(double bottom) {
        m_rightBottomTalon.set(ControlMode.Velocity, bottom);
        //System.out.println(m_rightBottomTalon.getSupplyCurrent());
    }

    public void shootPO(double bottom) {
        m_rightBottomTalon.set(ControlMode.PercentOutput, bottom);
    }

    public boolean isOn() {
        return m_rightBottomTalon.getSelectedSensorVelocity() == 0;
    }

    public boolean isReady() {
        return Math.abs(getErrorBottom()) < SubsystemConfig.SHOOTER_MAXIMUM_ALLOWED_ERROR;
    }

    public double getRateBottom() {
        return m_rightBottomTalon.getSelectedSensorVelocity() * /*Conversions.*/Units.DRIVETRAIN_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }

    public int getVelocityBottom() {
        return (int)m_rightBottomTalon.getSelectedSensorVelocity();
    }

    public int getErrorBottom() {
        return (int)m_rightBottomTalon.getClosedLoopError();
    }

    public double getTargetBottom() {
        return m_rightBottomTalon.getControlMode() == ControlMode.Velocity ? m_rightBottomTalon.getClosedLoopTarget() : 0;
    }

    private void configureMotors() {
        
        //First setup talons with default settings
        m_rightBottomTalon.configFactoryDefault();
        m_leftBottomTalon.configFactoryDefault();

        
        m_rightBottomTalon.setSensorPhase(false);

        // Driving shooter motors in different directions
        m_rightBottomTalon.setInverted(false);
        m_leftBottomTalon.setInverted(true);

        m_leftBottomTalon.follow(m_rightBottomTalon, MotorConfig.DEFAULT_MOTOR_FOLLOWER_TYPE);

        m_rightBottomTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        
        TalonSRXConfiguration cBottom = new TalonSRXConfiguration();

        cBottom.slot0 = MotionControl.SHOOTER_BOTTOM_PID;

        m_rightBottomTalon.setNeutralMode(NeutralMode.Brake);
        m_leftBottomTalon.setNeutralMode(NeutralMode.Brake);

        //Configure talons
        m_rightBottomTalon.configAllSettings(cBottom);
    }
    
}
