// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class followTargetCommand extends CommandBase {
  /** Creates a new followTargetCommand. */
  double areaSetPoint = 1.0;
  double areaKp = .1;
  double yawKp  = .1;

  public followTargetCommand(DriveTrainSubsystem driveTrain, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(vision);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(VisionSubsystem.hasTarget()){
      double areaError = areaSetPoint - VisionSubsystem.getArea();
      double yawError = 0 - VisionSubsystem.getYaw();
      DriveTrainSubsystem.setVelocities((areaError * areaKp) - (yawError * yawKp), (areaError * areaKp) + (yawError * yawKp));
    }else{
      DriveTrainSubsystem.setVelocities(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
