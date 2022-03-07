// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Projectile.Range;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Revs the shooter at the velocity automatically calculated by the Limelight
 */
public class RevShooterAtAutoVelocityCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  NetworkTableEntry vectorMapRange = NetworkTableInstance.getDefault().getTable("CommandoDash").getSubTable("SensorData").getEntry("vectorMapRange");

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
    Range range = shooterSubsystem.findRangeGivenDistance(shooterSubsystem.getHorizontalDistanceToHub());
    shooterSubsystem.setFlywheelTargetRpm(shooterSubsystem.calculateIdealLaunchVector().velocity);
    vectorMapRange.setString(range.minValue + " - " + range.maxValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.disableLimelightLed();
    shooterSubsystem.stop();
    vectorMapRange.setString("_._ - _._");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
