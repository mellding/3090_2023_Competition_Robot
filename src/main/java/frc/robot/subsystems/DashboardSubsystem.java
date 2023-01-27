// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardSubsystem extends SubsystemBase {
  /** Creates a new DashboardSubsystem. */
  private static SendableChooser<String> controlChooser;
  private final static String xbox = "xbox";
  private final static String joystick = "joystick";
  public DashboardSubsystem() {
    
    controlChooser = new SendableChooser<>();
    controlChooser.setDefaultOption("xbox", xbox);
    controlChooser.addOption("joystick", joystick);
    SmartDashboard.putData(controlChooser);
  }

  public static boolean xbox(){
    return controlChooser.getSelected() == "xbox";
  }

  public static boolean joystick(){
    return controlChooser.getSelected() == "joystick";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
