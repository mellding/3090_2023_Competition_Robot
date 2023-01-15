// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  private static final Pigeon2  pigeonIMU = new Pigeon2(Constants.DriveTrainConstants.PIGEAN_IMU_CAN_ID);
  private static final CANSparkMax leftPrimary = new CANSparkMax(Constants.DriveTrainConstants.LEFT_PRIMARY_CAN_ID, MotorType.kBrushless);
  private static final CANSparkMax leftSecondary = new CANSparkMax(Constants.DriveTrainConstants.LEFT_SECONDARY_CAN_ID, MotorType.kBrushless);
  private static final CANSparkMax rightPrimary = new CANSparkMax(Constants.DriveTrainConstants.RIGHT_PRIMARY_CAN_ID, MotorType.kBrushless);
  private static final CANSparkMax rightSecondary = new CANSparkMax(Constants.DriveTrainConstants.RIGHT_SECONDARY_CAN_ID, MotorType.kBrushless);
  private final  DifferentialDriveOdometry driveOdometry;
  private static DifferentialDrive diffDrive;
  private static RelativeEncoder leftEncoder;
  private static RelativeEncoder rightEncoder;
  private static SparkMaxPIDController leftPID;
  private static SparkMaxPIDController rightPID;

  private static double leftMaxCurrent = 0.0;
  private static double rightMaxCurrent = 0.0;

  public DriveTrainSubsystem() {
    leftPrimary.restoreFactoryDefaults();      leftSecondary.restoreFactoryDefaults();
    rightPrimary.restoreFactoryDefaults();      rightSecondary.restoreFactoryDefaults();

    leftPrimary.setIdleMode(IdleMode.kCoast);   leftSecondary.setIdleMode(IdleMode.kCoast);
    rightPrimary.setIdleMode(IdleMode.kCoast);  rightSecondary.setIdleMode(IdleMode.kCoast);

    leftPrimary.setInverted(false);leftSecondary.setInverted(false);
    rightPrimary.setInverted(true);rightSecondary.setInverted(true);

    leftSecondary.follow(leftPrimary); rightSecondary.follow(rightPrimary);

    leftPrimary.setOpenLoopRampRate(Constants.DriveTrainConstants.OPEN_LOOP_RAMP_RATE); 
    rightPrimary.setOpenLoopRampRate(Constants.DriveTrainConstants.OPEN_LOOP_RAMP_RATE);

    leftPrimary.setSmartCurrentLimit(Constants.DriveTrainConstants.MOTOR_CURRENT_LIMIT);
    leftSecondary.setSmartCurrentLimit(Constants.DriveTrainConstants.MOTOR_CURRENT_LIMIT);
    rightPrimary.setSmartCurrentLimit(Constants.DriveTrainConstants.MOTOR_CURRENT_LIMIT);
    rightSecondary.setSmartCurrentLimit(Constants.DriveTrainConstants.MOTOR_CURRENT_LIMIT);

    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    leftEncoder = leftPrimary.getEncoder(); 
    rightEncoder = rightPrimary.getEncoder();

    leftEncoder.setInverted(false); rightEncoder.setInverted(true);

    leftPID = leftPrimary.getPIDController();
    rightPID = rightPrimary.getPIDController();

    leftPID.setP(Constants.DriveTrainConstants.LEFT_KP);
    leftPID.setI(Constants.DriveTrainConstants.LEFT_KI);
    leftPID.setD(Constants.DriveTrainConstants.LEFT_KD);
    leftPID.setFF(Constants.DriveTrainConstants.LEFT_KFF);

    rightPID.setP(Constants.DriveTrainConstants.RIGHT_KP);
    rightPID.setI(Constants.DriveTrainConstants.RIGHT_KI);
    rightPID.setD(Constants.DriveTrainConstants.RIGHT_KD);
    rightPID.setFF(Constants.DriveTrainConstants.RIGHT_KFF);

    leftPrimary.burnFlash(); leftSecondary.burnFlash();
    rightPrimary.burnFlash(); rightSecondary.burnFlash();

    diffDrive = new DifferentialDrive(leftPrimary, rightPrimary);
    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()), getLeftPositionFt(), getRightPositionFt());
  }

  public static void GTA_Drive(double leftPower, double rightPower, double turn){
    setCoast();
    setMotors((rightPower - leftPower) - turn, (rightPower - leftPower) + turn);
  }

  public static void GTA_Drive_Slow(double leftPower, double rightPower, double turn){
    setCoast();
    double rightPowerSlow = mapDouble((rightPower - leftPower) + turn, -2, 2, 
                                        -Constants.DriveTrainConstants.SLOW_DRIVE_MAX, 
                                         Constants.DriveTrainConstants.SLOW_DRIVE_MAX);
    double leftPowerSlow = mapDouble((rightPower - leftPower) - turn, -2, 2, 
                                        -Constants.DriveTrainConstants.SLOW_DRIVE_MAX, 
                                         Constants.DriveTrainConstants.SLOW_DRIVE_MAX);
    setMotors(leftPowerSlow, rightPowerSlow);
  }

  public static void setMotors(double leftPower, double rightPower){
    leftPrimary.set(leftPower);   rightPrimary.set(rightPower);
  }

  public static void setCoast(){
    leftPrimary.setIdleMode(IdleMode.kCoast);   leftSecondary.setIdleMode(IdleMode.kCoast);
    rightPrimary.setIdleMode(IdleMode.kCoast);  rightSecondary.setIdleMode(IdleMode.kCoast);
    leftPrimary.setOpenLoopRampRate(Constants.DriveTrainConstants.OPEN_LOOP_RAMP_RATE); 
    rightPrimary.setOpenLoopRampRate(Constants.DriveTrainConstants.OPEN_LOOP_RAMP_RATE);
  }

  public static void setBrake(){
    leftPrimary.setIdleMode(IdleMode.kBrake);   leftSecondary.setIdleMode(IdleMode.kBrake);
    rightPrimary.setIdleMode(IdleMode.kBrake);  rightSecondary.setIdleMode(IdleMode.kBrake);
    leftPrimary.setOpenLoopRampRate(0); 
    rightPrimary.setOpenLoopRampRate(0);
    setMotors(0, 0);
  }

  public static double getLeftPositionFt(){
    return ((leftEncoder.getPosition() / Constants.DriveTrainConstants.MOTOR_ENCODER_CPR) 
                / Constants.DriveTrainConstants.DRIVE_GEAR_RATIO)
                * Constants.DriveTrainConstants.WHEEL_CIRCUMFRENCE_FT;
  }

  public static double getRightPositionFt(){
    return ((rightEncoder.getPosition() / Constants.DriveTrainConstants.MOTOR_ENCODER_CPR) 
                / Constants.DriveTrainConstants.DRIVE_GEAR_RATIO) 
                * Constants.DriveTrainConstants.WHEEL_CIRCUMFRENCE_FT;
  }
  
  public static double getLeftCurrent(){return (leftPrimary.getOutputCurrent() + leftSecondary.getOutputCurrent()) / 2;}
  public static double getRightCurrent(){return (rightPrimary.getOutputCurrent() + rightSecondary.getOutputCurrent()) / 2;}
  public static double getLeftMaxCurrent(){return leftMaxCurrent;}
  public static double getRightMaxCurrent(){return rightMaxCurrent;}
  public static double getYaw(){return pigeonIMU.getYaw();}
  public static double getPitch(){return pigeonIMU.getPitch();}
  public static double getRoll(){return pigeonIMU.getRoll();}

  private static double mapDouble(double valueIn, double baseMin, double baseMax, double limitMin, double limitMax) {
    return ((limitMax - limitMin) * (valueIn - baseMin) / (baseMax - baseMin)) + limitMin;
  }

  @Override
  public void periodic() {
    if(getLeftCurrent() > leftMaxCurrent) leftMaxCurrent = getLeftCurrent();
    if(getRightCurrent() > rightMaxCurrent) rightMaxCurrent = getRightCurrent();

    driveOdometry.update(Rotation2d.fromDegrees(getYaw()), getLeftPositionFt(), getRightPositionFt());
  }
}
