package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;


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
        m_topTalon.set(ControlMode.Velocity, top, DemandType.ArbitraryFeedForward, MotionControl.DRIVETRAIN_FEEDFORWARD.calculate(top));
        m_leftBottomTalon.set(ControlMode.Velocity, bottom, DemandType.ArbitraryFeedForward, MotionControl.DRIVETRAIN_FEEDFORWARD.calculate(bottom));
    }

    public void shootPO(double top, double bottom) {
        m_topTalon.set(ControlMode.PercentOutput, top);
        m_leftBottomTalon.set(ControlMode.PercentOutput, bottom);
    }


    public double getRateTop() {
        return m_topTalon.getSelectedSensorVelocity() * /*Conversions.*/Units.DRIVETRAIN_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }

    public double getRateBottom() {
        return m_leftBottomTalon.getSelectedSensorVelocity() * /*Conversions.*/Units.DRIVETRAIN_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }

    //@Log
    public int getVelocityTop() {
        return (int)m_topTalon.getSelectedSensorVelocity();
    }

    //@Log
    public int getVelocityBottom() {
        return (int)m_leftBottomTalon.getSelectedSensorVelocity();
    }

    //@Log
    public int getErrorTop() {
        return (int)m_topTalon.getClosedLoopError();
    }

    //@Log
    public int getErrorBottom() {
        return (int)m_leftBottomTalon.getClosedLoopError();
    }

    //@Log
    public double getTargetTop() {
        return m_topTalon.getControlMode() == ControlMode.Velocity ? m_topTalon.getClosedLoopTarget() : 0;
    }

    //@Log
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

        m_topTalon.setInverted(false);
        // Driving shooter prototype motors in different directions
        m_leftBottomTalon.setInverted(true);
        m_rightBottomTalon.setInverted(false);

        m_rightBottomTalon.follow(m_leftBottomTalon, MotorConfig.DEFAULT_MOTOR_FOLLOWER_TYPE);

        //Setup talon built-in PID
        m_topTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        m_leftBottomTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        //m_rightFrontTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        

        //Create config objects
        TalonSRXConfiguration cTop = new TalonSRXConfiguration(), cBottom = new TalonSRXConfiguration();

        //Setup config objects with desired values
        cTop.slot0 = MotionControl.SHOOTER_TOP_PID;
        cBottom.slot0 = MotionControl.SHOOTER_LEFT_BOTTOM_PID;

        //Not sure if the two below are strictly necessary
        // How quickly to apply the power
        //cLeft.closedloopRamp = SubsystemConfig.DRIVETRAIN_CLOSED_LOOP_RAMP;
        //cRight.closedloopRamp = SubsystemConfig.DRIVETRAIN_CLOSED_LOOP_RAMP;

        //Brake mode so no coasting
        m_topTalon.setNeutralMode(NeutralMode.Brake);
        m_leftBottomTalon.setNeutralMode(NeutralMode.Brake);
        m_rightBottomTalon.setNeutralMode(NeutralMode.Brake);

        //Configure talons
        m_topTalon.configAllSettings(cTop);
        m_leftBottomTalon.configAllSettings(cBottom);
    }
    
}