// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class defaultDriveCommand extends CommandBase {
  /** Creates a new defaultDriveCommand. */
  private DriveTrainSubsystem driveTrain;
  private DoubleSupplier left;
  private DoubleSupplier right;
  private DoubleSupplier turn;
  
  public defaultDriveCommand(DriveTrainSubsystem driveTrain, DoubleSupplier left, DoubleSupplier right, DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain= driveTrain;
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
    driveTrain.GTA_Drive(left.getAsDouble(), right.getAsDouble(), turn.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
