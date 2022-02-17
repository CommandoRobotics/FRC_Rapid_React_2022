// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoAimSubsystem;

public class AimUsingVisionCommand extends CommandBase {

  AutoAimSubsystem autoAimSubsystem;

  public AimUsingVisionCommand(AutoAimSubsystem autoAimSubsystem) {
    this.autoAimSubsystem = autoAimSubsystem;
    addRequirements(autoAimSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoAimSubsystem.enableLimelightLed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(autoAimSubsystem.isTargetSeen()) {
      autoAimSubsystem.setPan(autoAimSubsystem.calculatePanOutput(autoAimSubsystem.getLimelightXOffset()));
    } else {
      autoAimSubsystem.stopPan();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoAimSubsystem.disableLimelightLed();
    autoAimSubsystem.stopPan();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
