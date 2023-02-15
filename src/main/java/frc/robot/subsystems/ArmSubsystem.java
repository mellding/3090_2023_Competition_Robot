// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private static CANSparkMax tiltMotor = new CANSparkMax(14, MotorType.kBrushless);
  private static CANSparkMax extendMotor = new CANSparkMax(15, MotorType.kBrushless);
  private static CANSparkMax rotateMotor = new CANSparkMax(16, MotorType.kBrushed);

  private static Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  private static DoubleSolenoid grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  private static boolean grabberIsOpen;

  private static RelativeEncoder tiltEncoder;
  private static RelativeEncoder extendEncoder;
  private static Encoder rotateEncoder;

  private static double tiltSetpoint = 0;
  private static double tiltError = 0;

  private static double rotateSetpoint = 0;;
  private static double rotateError = 0;

  public ArmSubsystem() {
    tiltMotor.restoreFactoryDefaults();
    extendMotor.restoreFactoryDefaults();
    rotateMotor.restoreFactoryDefaults();

    tiltMotor.setIdleMode(IdleMode.kBrake);

    tiltMotor.setOpenLoopRampRate(ArmConstants.TILT_RAMP_RATE);

    tiltMotor.setSmartCurrentLimit(5);

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
    if(power != 0){
      tiltMotor.set(power);
      tiltSetpoint = tiltEncoder.getPosition();
    }else{
      tiltError = tiltSetpoint - tiltEncoder.getPosition();
      tiltMotor.set(tiltError * ArmConstants.tiltKp);
    }
  }

  public static void tiltPos(double setpoint){
    tiltSetpoint = setpoint;
    tiltError = setpoint - tiltEncoder.getPosition();
    tiltMotor.set(tiltError * ArmConstants.tiltKp);
  }

  public static void rotate(double power){
    if( (power > 0 && rotateEncoder.getDistance() >= ArmConstants.ROTATE_UPPER_LIMIT) ||
          (power < 0  && rotateEncoder.getDistance() <= ArmConstants.ROTATE_LOWER_LIMIT) ) return;

    if(power != 0){
      rotateMotor.set(power);
      rotateSetpoint = rotateEncoder.getDistance();
    }else{
      rotateError = rotateSetpoint - rotateEncoder.getDistance();
      rotateMotor.set(rotateError * ArmConstants.rotateKp);
    }
  }

  public static void rotatePos(double setPoint){
    rotateSetpoint = setPoint;
    rotateError = setPoint - rotateEncoder.getDistance();
    rotateMotor.set(rotateError * ArmConstants.rotateKp);
  }

  public static double getTiltSetpoint(){return tiltSetpoint;}
  public static double getTiltError(){return tiltError;}
  public static double getRotateSetpoint(){return rotateSetpoint;}
  public static double getRotateError(){return rotateError;}
  public static double getRotatePosition(){return rotateEncoder.getDistance();}
  public static boolean getGrabberOpen(){return grabberIsOpen;}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
