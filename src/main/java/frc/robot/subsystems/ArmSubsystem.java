// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  CANSparkMax tiltMotor = new CANSparkMax(14, MotorType.kBrushless);
  CANSparkMax extendMotor = new CANSparkMax(15, MotorType.kBrushless);
  CANSparkMax rotateMotor = new CANSparkMax(16, MotorType.kBrushed);

  public ArmSubsystem() {
    tiltMotor.restoreFactoryDefaults();
    extendMotor.restoreFactoryDefaults();
    rotateMotor.restoreFactoryDefaults();

    tiltMotor.setIdleMode(IdleMode.kBrake);
    extendMotor.setIdleMode(IdleMode.kBrake);
    rotateMotor.setIdleMode(IdleMode.kBrake);

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
