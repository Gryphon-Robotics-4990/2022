package frc.robot;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.vision.*;
import frc.robot.units.*;


public final class Constants {
    //TODO replace -1 ports with actual port numbers
    public static class Ports {
        //Laptop ports
        public static int PORT_JOYSTICK_DRIVE = 0;
        public static int PORT_JOYSTICK_OPERATOR = 1;

        //RoboRIO sensor ports
        public static SPI.Port SPI_PORT_GYRO = SPI.Port.kMXP;
        //Below is format for analog sensors
        //public static int PWM_NAME = -1;

        //Solenoid PCM ports
        public static int PCM_INTAKE_FORWARD = -1;
        public static int PCM_INTAKE_REVERSE = -1;
        public static int PCM_SHOOTER_FORWARD = -1;
        public static int PCM_SHOOTER_REVERSE = -1;

        //CAN Bus IDs
        public static int CAN_PDP = 0;
        public static int CAN_DRIVETRAIN_RIGHT_FRONT_TALONSRX = -1;
        public static int CAN_DRIVETRAIN_RIGHT_REAR_TALONSRX = -1;
        public static int CAN_DRIVETRAIN_LEFT_FRONT_TALONSRX = -1;
        public static int CAN_DRIVETRAIN_LEFT_REAR_TALONSRX = -1;
        public static int CAN_SHOOTER_TOP_TALONSRX = -1;
        public static int CAN_SHOOTER_LEFT_BOTTOM_TALONSRX = -1;
        public static int CAN_SHOOTER_RIGHT_BOTTOM_TALONSRX = -1;
        public static int CAN_PRESHOOTER_TALONSRX = -1;
    }

    public static class MotorConfig {
        //Talon information
        public static double TALON_ENCODER_RESOLUTION = 4096; // = EPR = CPR
        public static int TALON_TIMEOUT_MS = 5;
        public static int TALON_DEFAULT_PID_ID = 0;//0 is primary, 1 is auxilary
        public static TalonSRXFeedbackDevice TALON_DEFAULT_FEEDBACK_DEVICE = TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative;
        public static FollowerType DEFAULT_MOTOR_FOLLOWER_TYPE = FollowerType.PercentOutput;
    }

    public static class RobotMeasurements {
        //TODO find robot physical characteristics
        public static double DRIVETRAIN_TRACKWIDTH = -1;
        public static double DRIVETRAIN_WHEEL_RADIUS = -1;
        public static double DRIVETRAIN_WHEEL_RADIUS_METERS = -1;
        public static double DRIVETRAIN_TRACKWIDTH_METERS = -1;

        public static double SHOOTER_HEIGHT_METERS = -1;
        public static double SHOOTER_ANGLE_RADIANS = -1;

        public static double LIMELIGHT_HEIGHT_METERS = -1;
        public static double LIMELIGHT_ANGLE_RADIANS = -1;
    }
    
    public static class Units {
        //Base units
        Unit METER = new BaseUnit(Dimension.Length, 1d);
        Unit KILOMETER = new BaseUnit(Dimension.Length, METER.getScalar() * 1000d);
        Unit FEET = new BaseUnit(Dimension.Length, METER.getScalar() * 3.280839895d);

        Unit SECOND = new BaseUnit(Dimension.Time, 1d);
        Unit MINUTE = new BaseUnit(Dimension.Time, SECOND.getScalar() * 60d);
        Unit HOUR = new BaseUnit(Dimension.Time, MINUTE.getScalar() * 60d);
        Unit MILLISECOND = new BaseUnit(Dimension.Time, SECOND.getScalar() / 1000d);
        Unit ENCODER_TIME = new BaseUnit(Dimension.Time, MILLISECOND.getScalar() * 100d);
        
        Unit KILOGRAM = new BaseUnit(Dimension.Mass, 1d);

        Unit RADIAN = new BaseUnit(Dimension.Angle, 1d);
        Unit ROTATION = new BaseUnit(Dimension.Angle, RADIAN.getScalar() * 2d * Math.PI);
        Unit DEGREE = new BaseUnit(Dimension.Angle, ROTATION.getScalar() / 360d);
        Unit ENCODER_ANGLE = new BaseUnit(Dimension.Angle, ROTATION.getScalar() / MotorConfig.TALON_ENCODER_RESOLUTION);

        Unit AMPERE = new BaseUnit(Dimension.Current, 1d);

        //Compound units
        Unit ENCODER_ANGULAR_VELOCITY = new CompoundUnit(ENCODER_ANGLE, ENCODER_TIME);

        Unit METERS_PER_SECOND = new CompoundUnit(METER, SECOND);

        Unit METERS_PER_SECOND_2 = new CompoundUnit(METERS_PER_SECOND, SECOND);
        Unit NEWTON = new CompoundUnit(new Unit[] {KILOGRAM, METERS_PER_SECOND_2}, new Unit[] {});
        Unit JOULE = new CompoundUnit(new Unit[] {NEWTON, METER}, new Unit[] {});
        Unit COULOMB = new CompoundUnit(new Unit[] {AMPERE, SECOND}, new Unit[] {});
        Unit VOLTAGE = new CompoundUnit(JOULE, COULOMB);

        
        //TODO How to implement scalar multipliers and angular->velocity?
        
        //Old Code
        public static double ENCODER_VELOCITY_UNIT_TO_SECONDS = 0.1;//Encoder measures things in units per 0.1s
        public static double DRIVETRAIN_ENCODER_DISTANCE_TO_METERS = 1 / MotorConfig.TALON_ENCODER_RESOLUTION * 2 * Math.PI * RobotMeasurements.DRIVETRAIN_WHEEL_RADIUS_METERS;
        public static double DRIVETRAIN_ENCODER_VELOCITY_TO_METERS_PER_SECOND = DRIVETRAIN_ENCODER_DISTANCE_TO_METERS / ENCODER_VELOCITY_UNIT_TO_SECONDS;
        public static double METERS_PER_SECOND_TO_DRIVETRAIN_ENCODER_VELOCITY = 1 / DRIVETRAIN_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
        public static double SHOOTER_ENCODER_VELOCITY_TO_METERS_PER_SECOND = 0;
        //public static double DRIVETRAIN_FEEDFORWARD_TO_ENCODER_UNITS = 1;//TODO find this number
        //public static double SHOOTER_FEEDFORWARD_TO_ENCODER_UNITS = 1;//TODO find this number
    }

    public static class SubsystemConfig {

        //Drivetrain movement information
        public static double DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY = 3450;//Approx 4.03 meters per second
        public static double DRIVETRAIN_MAXIMUM_CRUISE_SPEED_METERS_PER_SECOND = 3.95;//Max is ~4
        public static double DRIVETRAIN_MAXIMUM_MOVEMENT_SPEED_METERS_PER_SECOND = DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY * /*Conversions.*/Units.DRIVETRAIN_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
        public static double DRIVETRAIN_CLOSED_LOOP_RAMP = 0.1; //seconds from 0 to full or full to 0

        //Shooter movement information
        public static double SHOOTER_MAXIMUM_TESTED_ENCODER_VELOCITY = 5000;//TODO find this number
        public static double SHOOTER_MAXIMUM_ALLOWED_VELOCITY_ERROR = 50;//TODO find this number
        public static double SHOOTER_MAXIMUM_ALLOWED_ANGULAR_ERROR_DEGREES = 0.1;//TODO find this number
    }

    public static class Vision {
        //TODO find these two
        public static double TARGET_HEIGHT_METERS = 0;

        public static ControlPoint[] CONTROL_POINTS = {
            new ControlPoint(0, 1),
            new ControlPoint(1, 1)
        };
    }
    
    public static class MotionControl {
        //PID
        public static TalonSRXGains DRIVETRAIN_LEFT_PID = new TalonSRXGains(0.2, 0.0033, 30);
        public static TalonSRXGains DRIVETRAIN_RIGHT_PID = new TalonSRXGains(0.2, 0.0033, 12);
        public static TalonSRXGains SHOOTER_TOP_PID = new TalonSRXGains(0, 0, 0);
        public static TalonSRXGains SHOOTER_LEFT_BOTTOM_PID = new TalonSRXGains(0, 0, 0);

        //Feedforward
        public static double DRIVETRAIN_FEEDFORWARD_KV_UNITS = 1 / 12 / MotorConfig.TALON_ENCODER_RESOLUTION * 10;
        public static double DRIVETRAIN_FEEDFORWARD_KS_UNITS = 1 / 12;
        public static SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(0.843 * DRIVETRAIN_FEEDFORWARD_KS_UNITS, 0.362 * DRIVETRAIN_FEEDFORWARD_KV_UNITS, 0);
        public static SimpleMotorFeedforward SHOOTER_FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);

        //The following two could possibly just be normal PID values
        public static TalonSRXGains LIMELIGHT_SHOOTER_PID = new TalonSRXGains(0, 0, 0);
        public static TalonSRXGains LIMELIGHT_DRIVETRAIN_PID = new TalonSRXGains(0, 0, 0);

        public static double LIMELIGHT_SHOOTER_KP = 0.5;
        public static double LIMELIGHT_SHOOTER_KI = 0;
        public static double LIMELIGHT_SHOOTER_KD = 0;

        public static double LIMELIGHT_DRIVETRAIN_KP = 0.5;
        public static double LIMELIGHT_DRIVETRAIN_KI = 0;
        public static double LIMELIGHT_DRIVETRAIN_KD = 0;

        public static double CLIMB_BALANCE_KP = 0.5;
        public static double CLIMB_BALANCE_KI = 0;
        public static double CLIMB_BALANCE_KD = 0;
    }

    //Miscellaneous
    public static double JOYSTICKF310_AXIS_DEADBAND = 0.05;
    public static double JOYSTICK_INPUT_EXPONENT = 5/3/*2*/;

    //Operation config
    //@Config(name = "Rotation Input Multiplier", tabName = "Op Configuration")
    public static double ARCADE_ROTATION_MULTIPLIER = 0.75;

    //@Config(name = "Speed Input Multiplier", tabName = "Op Configuration")
    public static double ARCADE_SPEED_MULTIPLIER = 1;

    //@Config(name = "Intake Motor Speed", tabName = "Op Configuration")
    public static double INTAKE_MOTOR_SPEED = 0.4;

    //@Config(name = "Hopper Motor Speed", tabName = "Op Configuration")
    public static double HOPPER_MOTOR_SPEED = 0.4;

    //@Config(name = "Storage Motor Speed", tabName = "Op Configuration")
    public static double STORAGE_MOTOR_SPEED = 0.8;

    //@Config(name = "Climb Motor Speed", tabName = "Op Configuration")
    public static double CLIMB_MOTOR_SPEED = 1;

    //@Config(name = "Inserter Motor Speed", tabName = "Op Configuration")
    public static double INSERTER_MOTOR_SPEED = 0.4;

    //Classes
    public static class TalonSRXGains extends SlotConfiguration {

        public TalonSRXGains(double kP, double kI, double kD) {
            super();
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }
}
