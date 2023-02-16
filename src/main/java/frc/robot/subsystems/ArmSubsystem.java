// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.straightHelpDrive;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private static CANSparkMax tiltMotor = new CANSparkMax(14, MotorType.kBrushless);
  private static CANSparkMax extendMotor = new CANSparkMax(15, MotorType.kBrushless);
  private static CANSparkMax rotateMotor = new CANSparkMax(16, MotorType.kBrushed);

  private static Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  private static DoubleSolenoid grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  private static AnalogInput hiPressure = new AnalogInput(0);
  private static AnalogInput loPressure = new AnalogInput(1);


  private static boolean grabberIsOpen;

  private static RelativeEncoder tiltEncoder;
  private static RelativeEncoder extendEncoder;
  private static Encoder rotateEncoder;

  private static double tiltSetpoint = 0;
  private static double tiltError = 0;

  private static double rotateSetpoint = 0;;
  private static double rotateError = 0;

  private static double extendSetpoint = 0;
  private static double extendError = 0;

  public ArmSubsystem() {
    tiltMotor.restoreFactoryDefaults();
    extendMotor.restoreFactoryDefaults();
    rotateMotor.restoreFactoryDefaults();

    tiltMotor.setIdleMode(IdleMode.kBrake);
    extendMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setIdleMode(IdleMode.kBrake);

    tiltMotor.setOpenLoopRampRate(ArmConstants.TILT_RAMP_RATE);
    extendMotor.setOpenLoopRampRate(ArmConstants.EXTEND_RAMP_RATE);
    rotateMotor.setOpenLoopRampRate(ArmConstants.ROTATE_RAMP_RATE);

    tiltMotor.setSmartCurrentLimit(ArmConstants.TILT_MAX_CURRENT);
    extendMotor.setSmartCurrentLimit(ArmConstants.EXTEND_MAX_CURRENT);
    rotateMotor.setSmartCurrentLimit(ArmConstants.ROTATE_MAX_CURRENT);

    tiltEncoder = tiltMotor.getEncoder();
    extendEncoder = extendMotor.getEncoder();
    rotateEncoder = new Encoder(0, 1);

    tiltMotor.burnFlash();

    compressor.enableDigital();
    grabberSolenoid.set(Value.kForward);

  }

  public static void grabberOpen(){
    grabberSolenoid.set(Value.kForward);
    grabberIsOpen = true;
  }

  public static void grabberClose(){
    grabberSolenoid.set(Value.kReverse);
    grabberIsOpen = false;
  }

  public static void tilt(double power){
    if( (power > 0 && getTiltDegrees() >= ArmConstants.TILT_UPPER_LIMIT)  ||
          (power < 0 && getTiltDegrees() <= ArmConstants.TILT_LOWER_LIMIT) ) return;

    if(power != 0){
      tiltMotor.set(power);
      tiltSetpoint = tiltEncoder.getPosition();
    }else{
      tiltError = tiltSetpoint - tiltEncoder.getPosition();
      tiltMotor.set(tiltError * ArmConstants.tiltKp);
    }
  }

  public static void setTiltPos(double setpoint){
    if(setpoint >= ArmConstants.TILT_UPPER_LIMIT ||
        setpoint <= ArmConstants.TILT_LOWER_LIMIT) return;
    tiltSetpoint = setpoint;
    tiltError = setpoint - getTiltDegrees();
    tiltMotor.set(tiltError * ArmConstants.tiltKp);
  }

  public static void rotate(double power){
    if( (power > 0 && getRotateDegrees() >= ArmConstants.ROTATE_UPPER_LIMIT) ||
          (power < 0  && getRotateDegrees() <= ArmConstants.ROTATE_LOWER_LIMIT) ) return;

    if(power != 0){
      rotateMotor.set(power);
      rotateSetpoint = rotateEncoder.getDistance();
    }else{
      rotateError = rotateSetpoint - rotateEncoder.getDistance();
      rotateMotor.set(rotateError * ArmConstants.rotateKp);
    }
  }

  public static void setRotatePos(double setPoint){
    if(setPoint >= ArmConstants.ROTATE_UPPER_LIMIT ||
        setPoint <= ArmConstants.ROTATE_LOWER_LIMIT) return;
    rotateSetpoint = setPoint;
    rotateError = setPoint - getRotateDegrees();
    rotateMotor.set(rotateError * ArmConstants.rotateKp);
  }

  public static double getTiltSetpoint(){return tiltSetpoint;}
  public static double getTiltError(){return tiltError;}
  public static double getTiltPosition(){return tiltEncoder.getPosition();}
  public static double getTiltDegrees(){return tiltEncoder.getPosition() / ArmConstants.TILT_COUNTS_PER_DEGREE;}
  public static double getTiltMotorTemp(){return tiltMotor.getMotorTemperature();}


  public static double getRotateSetpoint(){return rotateSetpoint;}
  public static double getRotateError(){return rotateError;}
  public static double getRotatePosition(){return rotateEncoder.getDistance();}
  public static double getRotateDegrees(){return rotateEncoder.getDistance() / ArmConstants.ROTATE_COUNTS_PER_DEGREE;}

  public static double getExtendSetpoint(){return extendSetpoint;}
  public static double getExtendError(){return extendError;}
  public static double getExtendPosition(){return extendEncoder.getPosition();}
  public static double getExtendDistance(){return extendEncoder.getPosition() / ArmConstants.EXTEND_COUNTS_PER_INCH;}
  public static double getExtendMotorTemp(){return extendMotor.getMotorTemperature();}

  public static boolean getGrabberOpen(){return grabberIsOpen;}
  public static double getHiPressure(){
    return 250 * (hiPressure.getVoltage() / 5) - 25;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tilt Temp", getTiltMotorTemp());
    SmartDashboard.putNumber("Extend Temp", getExtendMotorTemp());
  }
}
