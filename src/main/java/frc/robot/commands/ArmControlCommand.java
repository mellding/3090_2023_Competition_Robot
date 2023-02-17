// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmControlCommand extends CommandBase {
  /** Creates a new ArmControlCommand. */
  double rotate, tilt, extend;
  public ArmControlCommand(ArmSubsystem arm, DoubleSupplier rotate, DoubleSupplier tilt, DoubleSupplier extend) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rotate = rotate.getAsDouble();
    this.tilt   = tilt.getAsDouble();
    this.extend = extend.getAsDouble();

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmSubsystem.tilt(tilt);
    ArmSubsystem.rotate(rotate);
    ArmSubsystem.extend(extend);
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
