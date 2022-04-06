package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TurretManualCommand;

import static frc.robot.Constants.*;

import java.util.function.BooleanSupplier;

public class TurretSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_turretTalon;

    public TurretSubsystem() {
        m_turretTalon = new WPI_TalonSRX(Ports.CAN_TURRET_TALONSRX);

        configureMotors();
        // Zeroing turret from wherever we start it at
        m_turretTalon.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Turret Position", getPositionDegrees());
        SmartDashboard.putBoolean("Turret Manual", isTurretManual());
        SmartDashboard.putBoolean("Turret Ready", isReady());
    }

    public void setPosition(double position) {
        m_turretTalon.set(ControlMode.Position, position);
    }

    public void setPositionDegrees(double deg) {
        double ticks = deg * Units.DEGREE.to(Units.ENCODER_ANGLE) * RobotMeasurements.TURRET_MOTOR_REDUCTION;
        setPosition(ticks);
    }

    public void drivePO(double speed) {
        double factor = 0.5; // We want to limit rotation speed
        speed *= factor;
        m_turretTalon.set(ControlMode.PercentOutput, speed);
    }

    public boolean isReady() {
        return Math.abs(m_turretTalon.getClosedLoopError()) < SubsystemConfig.TURRET_MAXIMUM_ALLOWED_ERROR;
    }

    public boolean isTurretManual() {
        if (this.getCurrentCommand() == null) {
            return false;
        }
        return this.getCurrentCommand().getClass() == TurretManualCommand.class;
    }

    public double getPositionDegrees() {
        //return (m_turretTalon.getSelectedSensorPosition() * 360)/RobotMeasurements.TOTAL_TURRET_TALON_TICKS_REVOLUTION;
        double factor = 1;
        factor = Units.ENCODER_ANGLE.to(Units.DEGREE);
        return factor * (m_turretTalon.getSelectedSensorPosition() / (RobotMeasurements.TURRET_MOTOR_REDUCTION / 50));
    }

    public double getEncoderPosition() {
        return m_turretTalon.getSelectedSensorPosition();
    }

    public int getError() {
        return (int)m_turretTalon.getClosedLoopError();
    }

    public double getTargetPosition() {
        return m_turretTalon.getControlMode() == ControlMode.Position ? m_turretTalon.getClosedLoopTarget() : 0;
    }

    private void configureMotors() {
        
        m_turretTalon.configFactoryDefault();

        //Inverts the direction that the encoder reads
        // (clockwise being positive or counter-clockwise being positive)
        m_turretTalon.setSensorPhase(false);

        m_turretTalon.setInverted(false);

        // Having both inverted and sensor phase set to false seemed to work

        // Setup talon built-in PID
        m_turretTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        
        //Create config objects
        TalonSRXConfiguration c = new TalonSRXConfiguration();

        //Setup config objects with desired values
        c.slot0 = MotionControl.TURRET_PID;

        // Brake mode so no coasting
        m_turretTalon.setNeutralMode(NeutralMode.Brake);

        // Configure talons
        m_turretTalon.configAllSettings(c);
    }
}
