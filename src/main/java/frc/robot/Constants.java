package frc.robot;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.vision.*;
import frc.robot.units.*;


public final class Constants {
    public static class Ports {
        //Laptop ports
        public static int PORT_JOYSTICK_DRIVE = 0;
        public static int PORT_JOYSTICK_OPERATOR = 1;

        //RoboRIO sensor ports
        
        //TODO find breakbeam port
        public static int DIO_BREAKBEAM = 1;

        //Below is format for analog sensors
        //public static int PWM_NAME = -1;

        //CAN Bus IDs
        public static int CAN_PDP = 0;
        // CAN values for drivetrain
        public static int CAN_DRIVETRAIN_RIGHT_FRONT_TALONSRX = 1;
        public static int CAN_DRIVETRAIN_RIGHT_REAR_TALONSRX = 2;
        public static int CAN_DRIVETRAIN_LEFT_FRONT_TALONSRX = 3;
        public static int CAN_DRIVETRAIN_LEFT_REAR_TALONSRX = 4;

        public static int CAN_TURRET_TALONSRX = 6;

        public static int CAN_INTAKE_RIGHT_TALONSRX = 8;
        public static int CAN_PRESHOOTER_TALONSRX = 7;
        //
        public static int CAN_SHOOTER_TOP_TALONSRX = 10;//used to be 23
        public static int CAN_SHOOTER_LEFT_BOTTOM_TALONSRX = 11;
        public static int CAN_SHOOTER_RIGHT_BOTTOM_TALONSRX = 23;//used to be 10
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
        public static double DRIVETRAIN_WHEEL_RADIUS = 3;
        public static double DRIVETRAIN_WHEEL_RADIUS_METERS = DRIVETRAIN_WHEEL_RADIUS * Units.INCH.to(Units.METER);
        public static double DRIVETRAIN_TRACKWIDTH_METERS = -1;
        public static double TURRET_MOTOR_REDUCTION = 14; // x50 for versa gearbox, but turret is after the reduction
        public static double TALON_ENCODER_RESOLUTION = 4096;
        public static double TOTAL_TURRET_TALON_TICKS_REVOLOTION = TALON_ENCODER_RESOLUTION * TURRET_MOTOR_REDUCTION;

        public static double SHOOTER_HEIGHT_METERS = -1;
        public static double SHOOTER_ANGLE_RADIANS = -1;

        public static double LIMELIGHT_HEIGHT_METERS = -1;
        public static double LIMELIGHT_ANGLE_RADIANS = -1;
    }
    
    public static class Units {
        //Base units
        public static Unit METER = new BaseUnit(Dimension.Length, 1d);
        public static Unit KILOMETER = new BaseUnit(Dimension.Length, METER.getScalar() * 1000d);
        public static Unit FEET = new BaseUnit(Dimension.Length, METER.getScalar() * 3.280839895d);
        public static Unit INCH = new BaseUnit(Dimension.Length, FEET.getScalar() *(1/12));

        public static Unit SECOND = new BaseUnit(Dimension.Time, 1d);
        public static Unit MINUTE = new BaseUnit(Dimension.Time, SECOND.getScalar() * 60d);
        public static Unit HOUR = new BaseUnit(Dimension.Time, MINUTE.getScalar() * 60d);
        public static Unit MILLISECOND = new BaseUnit(Dimension.Time, SECOND.getScalar() / 1000d);
        public static Unit ENCODER_TIME = new BaseUnit(Dimension.Time, MILLISECOND.getScalar() * 100d);
        
        public static Unit KILOGRAM = new BaseUnit(Dimension.Mass, 1d);

        public static Unit RADIAN = new BaseUnit(Dimension.Angle, 1d);
        public static Unit REVOLUTION = new BaseUnit(Dimension.Angle, RADIAN.getScalar() * 2d * Math.PI);
        public static Unit DEGREE = new BaseUnit(Dimension.Angle, REVOLUTION.getScalar() / 360d);
        // Angle represented by encoder ticks, i.e 4096 ticks is a full revolution
        public static Unit ENCODER_ANGLE = new BaseUnit(Dimension.Angle, REVOLUTION.getScalar() / MotorConfig.TALON_ENCODER_RESOLUTION);

        public static Unit AMPERE = new BaseUnit(Dimension.Current, 1d);

        //Compound units
        public static Unit ENCODER_ANGULAR_VELOCITY = new CompoundUnit(ENCODER_ANGLE, ENCODER_TIME);

        public static Unit METERS_PER_SECOND = new CompoundUnit(METER, SECOND);

        public static Unit METERS_PER_SECOND_2 = new CompoundUnit(METERS_PER_SECOND, SECOND);
        public static Unit NEWTON = new CompoundUnit(new Unit[] {KILOGRAM, METERS_PER_SECOND_2}, new Unit[] {});
        public static Unit JOULE = new CompoundUnit(new Unit[] {NEWTON, METER}, new Unit[] {});
        public static Unit COULOMB = new CompoundUnit(new Unit[] {AMPERE, SECOND}, new Unit[] {});
        public static Unit VOLTAGE = new CompoundUnit(JOULE, COULOMB);

        
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
        public static double DRIVETRAIN_MAXIMUM_CRUISE_SPEED_METERS_PER_SECOND = 3.95;//Max is ~4. RECACLULATIONS SHOW THIS VALUE SHOULD BE 0.395
        public static double DRIVETRAIN_MAXIMUM_MOVEMENT_SPEED_METERS_PER_SECOND = DRIVETRAIN_MAXIMUM_TESTED_ENCODER_VELOCITY * /*Conversions.*/Units.ENCODER_ANGULAR_VELOCITY.to(Units.METERS_PER_SECOND);
        public static double DRIVETRAIN_CLOSED_LOOP_RAMP = 0.1; //seconds from 0 to full or full to 0
        public static double DRIVETRAIN_STOP_THRESHOLD = -1; //if the robot exceeds this speed, it is not safe to shoot

        public static double TURRET_MAXIMUM_ALLOWED_ERROR = 500;
        public static double SHOOTER_MAXIMUM_ALLOWED_ERROR = 200;
        // During auto, how far we can be off the correct PID position
        public static double DRIVETRAIN_MAXIMUM_ALLOWED_ERROR = 200;
    }

    public static class Vision {
        // 8 ft 8 in (freedom units) = 264 cm
        public static double TARGET_HEIGHT_METERS = 2.64;
        // The distance at which we need to switch to lobs (no top roller)
        // At distances where we would need a lob, the limelight can't see the target anyways
        public static double LOB_THRESHOLD = 10;
        public static ControlPoint[] CONTROL_POINTS = {
            //Add control points here

            // Dummy control points to be able to test code
            new ControlPoint(0, 0),
            new ControlPoint(3, 4),
            new ControlPoint(5, 2),
            new ControlPoint(7, 9),
        };
    }
    
    public static class MotionControl {
        //PID
        public static TalonSRXGains DRIVETRAIN_LEFT_PID = new TalonSRXGains(0.2, 0.0033, 12);
        public static TalonSRXGains DRIVETRAIN_RIGHT_PID = new TalonSRXGains(0.2, 0.0033, 12);
        public static TalonSRXGains SHOOTER_BOTTOM_PID = new TalonSRXGains(0.45, 0.0001, 25);
        public static TalonSRXGains TURRET_OLD_PID = new TalonSRXGains(1, 0.00035, 5); // didn't work during SFR
        public static TalonSRXGains TURRET_PID = new TalonSRXGains(1, 0.00005, 16); // Don't ask

        //Feedforward
        public static double DRIVETRAIN_FEEDFORWARD_KV_UNITS = 1 / 12 / MotorConfig.TALON_ENCODER_RESOLUTION * 10;
        public static double DRIVETRAIN_FEEDFORWARD_KS_UNITS = 1 / 12;
        public static SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(0.843 * DRIVETRAIN_FEEDFORWARD_KS_UNITS, 0.362 * DRIVETRAIN_FEEDFORWARD_KV_UNITS, 0);
        public static SimpleMotorFeedforward SHOOTER_FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);
        public static SimpleMotorFeedforward TURRET_FEEDFORWARD = new SimpleMotorFeedforward(0, 0, 0);
    }
 
    //Miscellaneous
    public static double JOYSTICKF310_AXIS_DEADBAND = 0.05;
    public static double JOYSTICK_THROTTLE_EXPONENT = 1;//11/8;
    public static double JOYSTICK_TURNING_EXPONENT = 11/8;
    public static double JOYSTICK_OPERATOR_EXPONENT = 2;

    //Operation config
    //@Config(name = "Rotation Input Multiplier", tabName = "Op Configuration")
    public static double ARCADE_ROTATION_MULTIPLIER = 0.5;

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
