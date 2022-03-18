package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveUtil;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DrivetrainSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_leftFrontTalon, m_rightFrontTalon;
    private final WPI_VictorSPX m_leftRearVictor, m_rightRearVictor;

    //@Config.NumberSlider(name = "OBLOG_TEST SPEED MULT", defaultValue = 1.1, min = 0, max = 2)
    public DrivetrainSubsystem() {
        m_leftFrontTalon = new WPI_TalonSRX(Ports.CAN_DRIVETRAIN_LEFT_FRONT_TALONSRX);
        m_leftRearVictor = new WPI_VictorSPX(Ports.CAN_DRIVETRAIN_LEFT_REAR_TALONSRX);
        m_rightFrontTalon = new WPI_TalonSRX(Ports.CAN_DRIVETRAIN_RIGHT_FRONT_TALONSRX);
        m_rightRearVictor = new WPI_VictorSPX(Ports.CAN_DRIVETRAIN_RIGHT_REAR_TALONSRX);

        configureMotors();

        m_leftFrontTalon.setSelectedSensorPosition(0);
        m_rightFrontTalon.setSelectedSensorPosition(0);
    }

    // Drive to a certain encoder position, used during auto
    public void driveToPosition(double pos) {
        m_leftFrontTalon.set(ControlMode.Position, pos);
        m_rightFrontTalon.set(ControlMode.Position, pos);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Right Velocity", getVelocityRight());
        SmartDashboard.putNumber("Left Velocity", getVelocityLeft());
        SmartDashboard.putBoolean("Drivetrain Stopped", drivetrainReadyToShoot());
    }
    //Assumes left and right are in encoder units per 100ms
    public void driveRaw(double left, double right) {
        //TODO Add acceleration to feedforward?
        m_leftFrontTalon.set(ControlMode.Velocity, left);
        m_rightFrontTalon.set(ControlMode.Velocity, right);
    }

    public void drivePO(double left, double right) {
        m_leftFrontTalon.set(ControlMode.PercentOutput, left);
        m_rightFrontTalon.set(ControlMode.PercentOutput, right);
        System.out.println(m_rightFrontTalon.getSupplyCurrent());
    }

    //Functions below are for 0-1
    public void tankDrive(double left, double right) {
        //Convert from value in range [0, 1] to raw encoder units
        left *= SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        right *= SubsystemConfig.DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY;
        driveRaw(left, right);
    }

    public void tankDrive(double[] speeds) {
        this.tankDrive(speeds[0], speeds[1]);
    }

    public void arcadeDrive(double speed, double rot) {
        this.tankDrive(DriveUtil.arcadeToTankDrive(speed, rot));
    }

    public void arcadeDrive(double[] speeds) {
        this.arcadeDrive(speeds[0], speeds[1]);
    }

    public double getDistanceLeft() {
        return m_leftFrontTalon.getSelectedSensorPosition() * /*Conversions.*/Units.DRIVETRAIN_ENCODER_DISTANCE_TO_METERS;
    }

    public double getDistanceRight() {
        return m_rightFrontTalon.getSelectedSensorPosition() * /*Conversions.*/Units.DRIVETRAIN_ENCODER_DISTANCE_TO_METERS;
    }

    public double getRateLeft() {
        return m_leftFrontTalon.getSelectedSensorVelocity() * /*Conversions.*/Units.DRIVETRAIN_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }

    public double getRateRight() {
        return m_rightFrontTalon.getSelectedSensorVelocity() * /*Conversions.*/Units.DRIVETRAIN_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }

    public boolean drivetrainReadyToShoot() {
        return Math.abs(getRateLeft()) < SubsystemConfig.DRIVETRAIN_STOP_THRESHOLD && Math.abs(getRateRight()) < SubsystemConfig.DRIVETRAIN_STOP_THRESHOLD;
    }

    public boolean isInPosition() {
        return Math.abs(getErrorLeft()) < SubsystemConfig.DRIVETRAIN_MAXIMUM_ALLOWED_ERROR && Math.abs(getErrorLeft()) < SubsystemConfig.DRIVETRAIN_MAXIMUM_ALLOWED_ERROR;
    }

    public int getVelocityRight() {
        return (int)m_rightFrontTalon.getSelectedSensorVelocity();
    }

    public int getVelocityLeft() {
        return (int)m_leftFrontTalon.getSelectedSensorVelocity();
    }

    public int getErrorLeft() {
        return (int)m_leftFrontTalon.getClosedLoopError();
    }

    public int getErrorRight() {
        return (int)m_rightFrontTalon.getClosedLoopError();
    }

    public double getTargetLeft() {
        return m_leftFrontTalon.getControlMode() == ControlMode.Velocity ? m_leftFrontTalon.getClosedLoopTarget() : 0;
    }

    public double getTargetRight() {
        return m_rightFrontTalon.getControlMode() == ControlMode.Velocity ? m_rightFrontTalon.getClosedLoopTarget() : 0;
    }

    private void configureMotors() {
        
        //First setup talons with default settings
        m_leftFrontTalon.configFactoryDefault();
        m_leftRearVictor.configFactoryDefault();
        m_rightFrontTalon.configFactoryDefault();
        m_rightRearVictor.configFactoryDefault();

        
        // TODO figure out whether we need to switch the encoder direction
        m_leftFrontTalon.setSensorPhase(true);
        m_rightFrontTalon.setSensorPhase(true);

        m_rightFrontTalon.setInverted(true);
        m_rightRearVictor.setInverted(true);
        

        m_leftRearVictor.follow(m_leftFrontTalon, MotorConfig.DEFAULT_MOTOR_FOLLOWER_TYPE);
        m_rightRearVictor.follow(m_rightFrontTalon, MotorConfig.DEFAULT_MOTOR_FOLLOWER_TYPE);

        //Setup talon built-in PID
        m_leftFrontTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        m_rightFrontTalon.configSelectedFeedbackSensor(MotorConfig.TALON_DEFAULT_FEEDBACK_DEVICE, MotorConfig.TALON_DEFAULT_PID_ID, MotorConfig.TALON_TIMEOUT_MS);
        

        //Create config objects
        TalonSRXConfiguration cLeft = new TalonSRXConfiguration(), cRight = new TalonSRXConfiguration();

        //Setup config objects with desired values
        cLeft.slot0 = MotionControl.DRIVETRAIN_LEFT_PID;
        cRight.slot0 = MotionControl.DRIVETRAIN_RIGHT_PID;

        //Not sure if the two below are strictly necessary
        // How quickly to apply the power
        //cLeft.closedloopRamp = SubsystemConfig.DRIVETRAIN_CLOSED_LOOP_RAMP;
        //cRight.closedloopRamp = SubsystemConfig.DRIVETRAIN_CLOSED_LOOP_RAMP;

        //Brake mode so no coasting
        m_leftFrontTalon.setNeutralMode(NeutralMode.Brake);
        m_leftRearVictor.setNeutralMode(NeutralMode.Brake);
        m_rightFrontTalon.setNeutralMode(NeutralMode.Brake);
        m_rightRearVictor.setNeutralMode(NeutralMode.Brake);

        //Configure talons
        m_leftFrontTalon.configAllSettings(cLeft);
        m_rightFrontTalon.configAllSettings(cRight);
    }
}
