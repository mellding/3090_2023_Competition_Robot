// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class straightHelpDrive extends CommandBase {
  /** Creates a new straightHelpDrive. */
  private DoubleSupplier left;
  private DoubleSupplier right;
  private DoubleSupplier turn;
  private double yawSetpoint;
  private double yawKP = 0.01;
  private double error;

  public straightHelpDrive(DriveTrainSubsystem driveTrain, DoubleSupplier left, DoubleSupplier right, DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.right = right;
    this.left = left;
    this.turn = turn;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(Math.abs(turn.getAsDouble()) > 0){
      error  = yawSetpoint - DriveTrainSubsystem.getYaw();
      DriveTrainSubsystem.setCoast();
      DriveTrainSubsystem.setVelocities((right.getAsDouble() - left.getAsDouble()) - (error * yawKP), 
                (right.getAsDouble() - left.getAsDouble()) + (error * yawKP));
    }else{
      DriveTrainSubsystem.setCoast();
      yawSetpoint = DriveTrainSubsystem.getYaw();
      DriveTrainSubsystem.setVelocities((right.getAsDouble() - left.getAsDouble()) - turn.getAsDouble(), 
                (right.getAsDouble() - left.getAsDouble()) + turn.getAsDouble());
    }

    SmartDashboard.putNumber("Turn Error", error);

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
