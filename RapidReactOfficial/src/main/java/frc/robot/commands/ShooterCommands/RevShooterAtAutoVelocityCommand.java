// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
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
    if(shooterSubsystem.isTargetSeen()) {
      Range range = shooterSubsystem.findRangeGivenDistance(shooterSubsystem.getHorizontalDistanceToHub());
      shooterSubsystem.setFlywheelTargetRpm(shooterSubsystem.calculateIdealLaunchVector().velocity);
      if(range != null) {
        vectorMapRange.setString(range.minValue + " - " + range.maxValue);
      } else {
        vectorMapRange.setString("0.0 - 0.0");
      }
    } else {
      vectorMapRange.setString("0.0 - 0.0");
      shooterSubsystem.setFlywheelTargetRpm(ConstantsValues.noTargetRpm);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.disableLimelightLed();
    shooterSubsystem.stop();
    vectorMapRange.setString("0.0 - 0.0");
    if(interrupted) {
      
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
