// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveTrainConstants {

    public static final int PIGEAN_IMU_CAN_ID             = 9;
    public static final int LEFT_PRIMARY_CAN_ID           = 10;
    public static final int LEFT_SECONDARY_CAN_ID         = 11;
    public static final int RIGHT_PRIMARY_CAN_ID          = 12;
    public static final int RIGHT_SECONDARY_CAN_ID        = 13;
    public static final int MOTOR_ENCODER_CPR             = 42;
    public static final int MOTOR_CURRENT_LIMIT           = 40;
    public static final int MOTOR_MAX_RPM                 = 5600;

    public static final double OPEN_LOOP_RAMP_RATE        = 1.0;
    public static final double SLOW_DRIVE_MAX             = 0.375;
    public static final double WHEEL_DIAMETER_IN          = 6.0;
    public static final double WHEEL_DIAMETER_FT          = WHEEL_DIAMETER_IN / 12.0;
    public static final double WHEEL_CIRCUMFRENCE_IN      = Math.PI * WHEEL_DIAMETER_IN;
    public static final double DRIVE_GEAR_RATIO           = 8.46;
    public static final double WHEEL_CPR                  = MOTOR_ENCODER_CPR * DRIVE_GEAR_RATIO;
    public static final double DRIVE_COUNTS_PER_INCH      = WHEEL_CPR / WHEEL_CIRCUMFRENCE_IN;
    public static final double WHEEL_CIRCUMFRENCE_FT      = Math.PI * WHEEL_DIAMETER_FT;
    public static final double DRIVE_COUNTS_PER_FT        = DRIVE_COUNTS_PER_INCH * 12;
    public static final double DRIVE_TRAIN_WIDTH          = 27;//????
    public static final double ENCODER_CONVERSION_FACTOR  = 1/DRIVE_GEAR_RATIO * WHEEL_CIRCUMFRENCE_FT;

    public static final double TURN_POWER_SCALAR          = 4;

    public static final double LEFT_KP                    = 0.01;
    public static final double LEFT_KI                    = 0.0;
    public static final double LEFT_KD                    = 0.0;
    public static final double LEFT_KFF                   = 0.0;
    
    public static final double RIGHT_KP                   = 0.01;
    public static final double RIGHT_KI                   = 0.0;
    public static final double RIGHT_KD                   = 0.0;
    public static final double RIGHT_KFF                  = 0.0;
    
    public static final double PID_MAX                    = 1.0;
    public static final double CLOSED_LOOP_RAMP           = 2.0;

    public static final double turnKp                     = .01;
  }

  public static class ArmConstants{
    public static final double ROTATE_ENCODER_CPR            = 40;
    public static final double ROTATE_GEAR_RATIO             = 64;
    public static final double ROTATE_CPR                    = ROTATE_ENCODER_CPR * ROTATE_GEAR_RATIO;
    public static final double ROTATE_COUNTS_PER_DEGREE      = ROTATE_CPR / 360;
    public static final double ROTATE_UPPER_LIMIT            = 180;
    public static final double ROTATE_LOWER_LIMIT            = -180;

    public static final double TILT_GEAR_RATIO               = 100;
    public static final double TILT_ENCODER_CPR              = 1;
    public static final double TILT_CPR                      = TILT_ENCODER_CPR * TILT_GEAR_RATIO;
    public static final double TILT_COUNTS_PER_DEGREE        = TILT_CPR / 360;
    public static final double TILT_UPPER_LIMIT              = 90;
    public static final double TILT_LOWER_LIMIT              = 0;

    public static final double EXTEND_GEAR_RATIO             = 16;
    public static final double EXTEND_COUNTS_PER_INCH        = (1.754 * Math.PI) * EXTEND_GEAR_RATIO;
    public static final double EXTENT_UPPER_LIMIT            = 18;
    public static final double EXTEND_LOWER_LIMIT            = 0;

    public static final double TILT_RAMP_RATE                = .5;
    public static final double ROTATE_RAMP_RATE              = .5;
    public static final double EXTEND_RAMP_RATE              = .5;

    public static final int TILT_MAX_CURRENT                 = 5;
    public static final int ROTATE_MAX_CURRENT               = 5;
    public static final int EXTEND_MAX_CURRENT               = 5;

    public static final double tiltKp                        = .01;
    public static final double rotateKp                      = .01;
    public static final double extentKp                      = .01;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

}
