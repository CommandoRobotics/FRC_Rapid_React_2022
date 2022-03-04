// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAimAutonomousCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  AutoAimSubsystem autoAimSubsystem;
  boolean isFinished = false;

  /** Creates a new AutoAimAutonomousCommand. */
  public AutoAimAutonomousCommand(DriveSubsystem driveSubsystem, AutoAimSubsystem autoAimSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.autoAimSubsystem = autoAimSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoAimSubsystem.enableLimelightLed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate pan output
    double panOutput = autoAimSubsystem.calculatePanOutput(autoAimSubsystem.getLimelightXOffset());
    // Determine if this command needs to end. This happens if the pan output is 0
    // which means that the drivetrain has reached its target.
    isFinished = panOutput == 0;

    // Drive the robot using the given pan output
    driveSubsystem.driveMecanum(0, 0, panOutput);
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
    return isFinished;
  }
}
