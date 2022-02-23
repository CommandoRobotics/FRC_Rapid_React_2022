// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Revs the shooter at the velocity automatically calculated by the Limelight
 */
public class RevShooterAtAutoVelocityCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;

  /** Creates a new RevShooterAtAutoVelocityCommand. */
  public RevShooterAtAutoVelocityCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.enableLimelightLed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.enableLimelightLed();
    shooterSubsystem.setBottomTargetRpm(
      shooterSubsystem.calculateIdealLaunchVector().velocity
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.disableLimelightLed();
    shooterSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
