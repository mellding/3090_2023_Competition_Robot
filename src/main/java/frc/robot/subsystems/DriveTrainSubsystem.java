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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  private static final Pigeon2  pigeonIMU = new Pigeon2(DriveTrainConstants.PIGEAN_IMU_CAN_ID);
  private static final CANSparkMax leftPrimary = new CANSparkMax(DriveTrainConstants.LEFT_PRIMARY_CAN_ID, MotorType.kBrushless);
  private static final CANSparkMax leftSecondary = new CANSparkMax(DriveTrainConstants.LEFT_SECONDARY_CAN_ID, MotorType.kBrushless);
  private static final CANSparkMax rightPrimary = new CANSparkMax(DriveTrainConstants.RIGHT_PRIMARY_CAN_ID, MotorType.kBrushless);
  private static final CANSparkMax rightSecondary = new CANSparkMax(DriveTrainConstants.RIGHT_SECONDARY_CAN_ID, MotorType.kBrushless);
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

    leftPrimary.setOpenLoopRampRate(DriveTrainConstants.OPEN_LOOP_RAMP_RATE); 
    rightPrimary.setOpenLoopRampRate(DriveTrainConstants.OPEN_LOOP_RAMP_RATE);
    leftPrimary.setClosedLoopRampRate(DriveTrainConstants.CLOSED_LOOP_RAMP);
    rightPrimary.setClosedLoopRampRate(DriveTrainConstants.CLOSED_LOOP_RAMP);

    leftPrimary.setSmartCurrentLimit(DriveTrainConstants.MOTOR_CURRENT_LIMIT);
    leftSecondary.setSmartCurrentLimit(DriveTrainConstants.MOTOR_CURRENT_LIMIT);
    rightPrimary.setSmartCurrentLimit(DriveTrainConstants.MOTOR_CURRENT_LIMIT);
    rightSecondary.setSmartCurrentLimit(DriveTrainConstants.MOTOR_CURRENT_LIMIT);

    
    leftEncoder = leftPrimary.getEncoder(); rightEncoder = rightPrimary.getEncoder();
    
    leftEncoder.setInverted(false); rightEncoder.setInverted(true);
    
    leftEncoder.setPosition(0.0); rightEncoder.setPosition(0.0);
    
    leftEncoder.setPositionConversionFactor(DriveTrainConstants.ENCODER_CONVERSION_FACTOR);
    rightEncoder.setPositionConversionFactor(DriveTrainConstants.ENCODER_CONVERSION_FACTOR);
    leftEncoder.setVelocityConversionFactor(DriveTrainConstants.ENCODER_CONVERSION_FACTOR/60);
    rightEncoder.setVelocityConversionFactor(DriveTrainConstants.ENCODER_CONVERSION_FACTOR/60);

    leftPID = leftPrimary.getPIDController();
    rightPID = rightPrimary.getPIDController();

    leftPID.setP(DriveTrainConstants.LEFT_KP);
    leftPID.setI(DriveTrainConstants.LEFT_KI);
    leftPID.setD(DriveTrainConstants.LEFT_KD);
    leftPID.setFF(DriveTrainConstants.LEFT_KFF);
    leftPID.setOutputRange(-DriveTrainConstants.PID_MAX, DriveTrainConstants.PID_MAX);

    rightPID.setP(DriveTrainConstants.RIGHT_KP);
    rightPID.setI(DriveTrainConstants.RIGHT_KI);
    rightPID.setD(DriveTrainConstants.RIGHT_KD);
    rightPID.setFF(DriveTrainConstants.RIGHT_KFF);
    rightPID.setOutputRange(-DriveTrainConstants.PID_MAX, DriveTrainConstants.PID_MAX);

    leftPrimary.burnFlash(); leftSecondary.burnFlash();
    rightPrimary.burnFlash(); rightSecondary.burnFlash();

    diffDrive = new DifferentialDrive(leftPrimary, rightPrimary);
    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()), 
                                                    leftEncoder.getPosition(), 
                                                    rightEncoder.getPosition());

    driveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), 
                                  leftEncoder.getPosition(), 
                                  rightEncoder.getPosition(), 
                                  new Pose2d());
  }

  public void GTA_Drive(double leftPower, double rightPower, double turn){
    setCoast();
    setMotors((rightPower - leftPower) - turn, (rightPower - leftPower) + turn);
  }

  public static void GTA_Drive_Slow(double leftPower, double rightPower, double turn){
    setCoast();
    double rightPowerSlow = mapDouble((rightPower - leftPower) + turn, -2, 2, 
                                        -DriveTrainConstants.SLOW_DRIVE_MAX, 
                                         DriveTrainConstants.SLOW_DRIVE_MAX);
    double leftPowerSlow = mapDouble((rightPower - leftPower) - turn, -2, 2, 
                                        -DriveTrainConstants.SLOW_DRIVE_MAX, 
                                         DriveTrainConstants.SLOW_DRIVE_MAX);
    setMotors(leftPowerSlow, rightPowerSlow);
  }

  public static void setMotors(double leftPower, double rightPower){
    leftPrimary.set(leftPower);   rightPrimary.set(rightPower);
  }

  public static void setCoast(){
    leftPrimary.setIdleMode(IdleMode.kCoast);   leftSecondary.setIdleMode(IdleMode.kCoast);
    rightPrimary.setIdleMode(IdleMode.kCoast);  rightSecondary.setIdleMode(IdleMode.kCoast);
    leftPrimary.setOpenLoopRampRate(DriveTrainConstants.OPEN_LOOP_RAMP_RATE); 
    rightPrimary.setOpenLoopRampRate(DriveTrainConstants.OPEN_LOOP_RAMP_RATE);
  }

  public static void setBrake(){
    leftPrimary.setIdleMode(IdleMode.kBrake);   leftSecondary.setIdleMode(IdleMode.kBrake);
    rightPrimary.setIdleMode(IdleMode.kBrake);  rightSecondary.setIdleMode(IdleMode.kBrake);
    leftPrimary.setOpenLoopRampRate(0); 
    rightPrimary.setOpenLoopRampRate(0);
    setMotors(0, 0);
  }
  
  public static double getLeftPosition(){return leftEncoder.getPosition();}
  public static double getRightPosition(){return rightEncoder.getPosition();}
  public static double getLeftVelocity(){return leftEncoder.getVelocity();}
  public static double getRightVelovity(){return rightEncoder.getVelocity();}
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

    driveOdometry.update(Rotation2d.fromDegrees(getYaw()), leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putNumber("Left Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", rightEncoder.getPosition());
    SmartDashboard.putNumber("Yaw", pigeonIMU.getYaw());

    
  }
}
