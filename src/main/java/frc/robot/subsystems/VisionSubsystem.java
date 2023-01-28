// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static boolean hasTarget;
  private static double y;
  private static double x;
  private static double area;
  private static NetworkTable table;
  private static NetworkTableEntry tv;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry ta;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");    
  }
  
  public static boolean hasTarget(){return hasTarget;}
  public static double getPitch(){return y;}
  public static double getYaw(){return x;}
  public static double getArea(){return area;}

  @Override
  public void periodic() {
    //read values periodically
    hasTarget = tv.getBoolean(false);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putBoolean("Has Target", hasTarget);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}
