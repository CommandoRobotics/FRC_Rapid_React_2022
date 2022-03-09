// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveFieldCentricCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  DoubleSupplier y, x, rotation;

  /** Creates a new DriveFieldCentric. */
  public DriveFieldCentricCommand(DriveSubsystem driveSubsystem, DoubleSupplier y, DoubleSupplier x, DoubleSupplier rotation) {
    this.driveSubsystem = driveSubsystem;
    this.y = y;
    this.x = x;
    this.rotation = rotation;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.driveMecanum(y.getAsDouble(), x.getAsDouble(), rotation.getAsDouble(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
