package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.DriveUtil;
import frc.robot.vision.Limelight;
import static frc.robot.Constants.*;

public class TurretSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_turretTalon;

    //@Config.NumberSlider(name = "OBLOG_TEST SPEED MULT", defaultValue = 1.1, min = 0, max = 2)
    public TurretSubsystem() {
        m_turretTalon = new WPI_TalonSRX(Ports.CAN_TURRET_TALONSRX);

        configureMotors();

        m_turretTalon.setSelectedSensorPosition(0);
    }

    //Assumes left and right are in encoder units per 100ms
    public void driveRaw(double speed) {
        m_turretTalon.set(ControlMode.Position, right, DemandType.ArbitraryFeedForward, MotionControl.TURRET_FEEDFORWARD.calculate(right));
    }

    public void drivePO(double speed) {
        m_turretTalon.set(ControlMode.PercentOutput, right);
    }

    public boolean isReady() {
        return Math.abs(Limelight.getCrosshairHorizontalOffset()) < SubsystemConfig.TURRET_MAXIMUM_ALLOWED_ERROR;
    }

    public double getPositionDegrees() {
        return (m_turretTalon.getSelectedSensorPosition() * 360)/RobotMeasurements.TOTAL_TURRET_TALON_TICKS_REVOLOTION;
    }

    public int getError() {
        return (int)m_turretTalon.getClosedLoopError();
    }

    public double getTargetPosition() {
        return m_turretTalon.getControlMode() == ControlMode.Velocity ? m_turretTalon.getClosedLoopTarget() : 0;
    }

    private void configureMotors() {
        
        m_turretTalon.configFactoryDefault();

        //I have no clue what this does but it seems important
        m_turretTalon.setSensorPhase(true);

        //Motor inversion?
        m_turretTalon.setInverted(false);

        //Setup talon built-in PID
        m_turretTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        
        //Create config objects
        TalonSRXConfiguration c = new TalonSRXConfiguration()

        //Setup config objects with desired values
        c.slot0 = MotionControl.TURRET_PID;

        //Brake mode so no coasting
        m_turretTalon.setNeutralMode(NeutralMode.Brake);

        //Configure talons
        m_turretTalon.configAllSettings(c);
    }
}
