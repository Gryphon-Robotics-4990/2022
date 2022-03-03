package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import frc.robot.vision.Limelight;
import io.github.oblarg.oblog.annotations.Log;


public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_topTalon, m_leftBottomTalon, m_rightBottomTalon;

    public ShooterSubsystem() {
        //The top and left bottom talons use their own PID, but the bottom right just follows the bottom left.
        m_topTalon = new WPI_TalonSRX(Ports.CAN_SHOOTER_TOP_TALONSRX);
        m_leftBottomTalon = new WPI_TalonSRX(Ports.CAN_SHOOTER_LEFT_BOTTOM_TALONSRX);
        m_rightBottomTalon = new WPI_TalonSRX(Ports.CAN_SHOOTER_RIGHT_BOTTOM_TALONSRX);

        configureMotors();
    }

    public void shootPID(double top, double bottom) {
        m_topTalon.set(ControlMode.Velocity, top, DemandType.ArbitraryFeedForward, MotionControl.SHOOTER_FEEDFORWARD.calculate(top));
        m_leftBottomTalon.set(ControlMode.Velocity, bottom, DemandType.ArbitraryFeedForward, MotionControl.SHOOTER_FEEDFORWARD.calculate(bottom));
    }

    public void shootPO(double top, double bottom) {
        m_topTalon.set(ControlMode.PercentOutput, top);
        m_leftBottomTalon.set(ControlMode.PercentOutput, bottom);
    }

    @Log(name = "Shooter Ready")
    public Boolean isReady() {
        return Math.abs(Limelight.getCrosshairHorizontalOffset()) < SubsystemConfig.SHOOTER_MAXIMUM_ALLOWED_ERROR;
    }

    @Log
    public boolean isTopRunning() {
        return m_topTalon.getMotorOutputPercent() > 0.0;
    }

    public double getRateBottom() {
        return m_leftBottomTalon.getSelectedSensorVelocity() * /*Conversions.*/Units.DRIVETRAIN_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }

    //@Log
    public int getVelocityBottom() {
        return (int)m_leftBottomTalon.getSelectedSensorVelocity();
    }

    //@Log
    public int getErrorBottom() {
        return (int)m_leftBottomTalon.getClosedLoopError();
    }

    @Log
    public double getTargetBottom() {
        return m_leftBottomTalon.getControlMode() == ControlMode.Velocity ? m_leftBottomTalon.getClosedLoopTarget() : 0;
    }

    private void configureMotors() {
        
        //First setup talons with default settings
        m_topTalon.configFactoryDefault();
        m_leftBottomTalon.configFactoryDefault();
        m_rightBottomTalon.configFactoryDefault();

        
        //Left side encoder goes in the wrong direction
        m_topTalon.setSensorPhase(true);
        m_leftBottomTalon.setSensorPhase(true);

        m_topTalon.setInverted(true);
        m_leftBottomTalon.setInverted(false);
        m_rightBottomTalon.setInverted(false);

        m_rightBottomTalon.follow(m_leftBottomTalon, MotorConfig.DEFAULT_MOTOR_FOLLOWER_TYPE);

        m_topTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        m_leftBottomTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        
        TalonSRXConfiguration cTop = new TalonSRXConfiguration(), cBottom = new TalonSRXConfiguration();

        cTop.slot0 = MotionControl.SHOOTER_TOP_PID;
        cBottom.slot0 = MotionControl.SHOOTER_LEFT_BOTTOM_PID;

        m_topTalon.setNeutralMode(NeutralMode.Brake);
        m_leftBottomTalon.setNeutralMode(NeutralMode.Brake);
        m_rightBottomTalon.setNeutralMode(NeutralMode.Brake);

        //Configure talons
        m_topTalon.configAllSettings(cTop);
        m_leftBottomTalon.configAllSettings(cBottom);
    }
    
}
