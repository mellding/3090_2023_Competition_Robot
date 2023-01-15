// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static final PhotonCamera limeLight = new PhotonCamera("photonvision");
  private static boolean hasTarget;
  private static double pitch;
  private static double yaw;
  private static double area;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  public boolean hasTargets(){
    return hasTarget;
  }

  public double getPitch(){
    return pitch;
  }

  public double getYaw(){
    return yaw;
  }

  public double getArea(){
    return area;
  }

  @Override
  public void periodic() {
    var results = limeLight.getLatestResult();

    hasTarget = results.hasTargets();
    if(hasTarget){
      PhotonTrackedTarget target = results.getBestTarget();
      pitch = target.getPitch();
      yaw = target.getYaw();
      area = target.getArea();
    }
  }
}
