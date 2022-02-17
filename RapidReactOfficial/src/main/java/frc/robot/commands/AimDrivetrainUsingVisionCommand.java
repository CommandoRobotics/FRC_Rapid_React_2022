// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AimDrivetrainUsingVisionCommand extends CommandBase {

  DoubleSupplier x, y, rotation;
  DriveSubsystem driveSubsystem;
  AutoAimSubsystem autoAimSubsystem;

  /**
   * An AimDrivetrainUsingVision command
   * @param x The x input from the driver controller
   * @param y The y input from the driver controller
   * @param z The z input from the driver controller
   * @param driveSubsystem The drive subsystem
   * @param autoAimSubsystem The auto aim subsystem
   */
  public AimDrivetrainUsingVisionCommand(DoubleSupplier y, DoubleSupplier x, DoubleSupplier rotation, DriveSubsystem driveSubsystem, AutoAimSubsystem autoAimSubsystem) {
    this.y = y;
    this.x = x;
    this.rotation = rotation;
    addRequirements(driveSubsystem);
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
      // Automatically rotate the robot to the target
      driveSubsystem.driveMecanum(
        // Forward/reverse value
        y.getAsDouble(), 
        // Strafe value
        x.getAsDouble(),
        // Calculate rotation value using PID and Limelight offset 
        autoAimSubsystem.calculatePanOutput(
          autoAimSubsystem.getLimelightXOffset()
        ),
        true
        );
    } else {
      // Drive based on driver commands since we can't see the target
      driveSubsystem.driveMecanum(
        y.getAsDouble(), 
        x.getAsDouble(), 
        rotation.getAsDouble(), 
        true
        );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoAimSubsystem.disableLimelightLed();
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
