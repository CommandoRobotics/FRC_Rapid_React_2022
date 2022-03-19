// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.Projectile.Range;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This command revs the shooter to an automatically determined ideal velocity. Because this command is meant
 * to be implemented in an autonomous, it will end when the shooter reaches its target velocity and it will NOT
 * stop the shooter motors unless it is interrupted.
 */
public class RevShooterAtAutoVelocityNoStopCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  NetworkTableEntry vectorMapRange = NetworkTableInstance.getDefault().getTable("CommandoDash").getSubTable("SensorData").getEntry("vectorMapRange");

  /** Creates a new RevShooterAtAutoVelocityAutonomousCommand. */
  public RevShooterAtAutoVelocityNoStopCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.enableLimelightLed();
    shooterSubsystem.startTrackingReadiness();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSubsystem.isTargetSeen()) {
      shooterSubsystem.startTrackingReadiness(); // The RPM is deliberately chosen and therefore we are ready to shoot when that RPM is reached
      shooterSubsystem.enableLimelightLed();
      shooterSubsystem.setFlywheelTargetRpm(shooterSubsystem.calculateIdealLaunchVector().velocity);
      Range range = shooterSubsystem.findRangeGivenDistance(shooterSubsystem.getHorizontalDistanceToHub());
      if(range != null) {
        vectorMapRange.setString(range.minValue + " - " + range.maxValue);
      } else {
        vectorMapRange.setString("0.0 - 0.0");
      }
    } else {
      shooterSubsystem.stopTrackingReadiness(); // THe RPM is only chosen because we don't have a target. Therefore, we are not ready to shoot regardless of whether that RPM is reached.
      shooterSubsystem.setFlywheelTargetRpm(ConstantsValues.noTargetRpm);
      vectorMapRange.setString("0.0 - 0.0");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      shooterSubsystem.disableLimelightLed();
      shooterSubsystem.stop();
      vectorMapRange.setString("0.0 - 0.0");
      shooterSubsystem.stopTrackingReadiness();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    This command should be considered finished in two scenarios:
    1. The shooter reports that it has reached its target velocity
    2. The max runtime of this command is overrun
    */
    return false;
  }
}
