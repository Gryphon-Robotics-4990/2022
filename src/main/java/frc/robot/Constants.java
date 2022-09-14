// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Multipliers
    public static double driveSpeedMultiplier = 0.5;
    public static double rotationSpeedMultiplier = 0.4;

    //Device IDs
    public static int LEFT_FRONT_TALON_ID = 10;
    public static int LEFT_REAR_TALON_ID = 5;
    public static int RIGHT_FRONT_TALON_ID = 13;
    public static int RIGHT_REAR_TALON_ID = 15;

    //PID Variables
    public static double leftP = 0;
    public static double leftI = 0;
    public static double leftD = 0;
    public static double rightP = 0;
    public static double rightI = 0;
    public static double rightD = 0;

    //PID Controllers
    public static TalonSRXGains DRIVETRAIN_LEFT_PID = new TalonSRXGains(leftP, leftI, leftD);
    public static TalonSRXGains DRIVETRAIN_RIGHT_PID = new TalonSRXGains(rightP, rightI, rightD);

    

    //Talon Configuration
    public static double TALON_ENCODER_RESOLUTION = 4096;
    public static int TALON_DEFAULT_PID_ID = 0;
    public static TalonSRXFeedbackDevice TALON_DEFAULT_FEEDBACK_DEVICE = TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative;
}
