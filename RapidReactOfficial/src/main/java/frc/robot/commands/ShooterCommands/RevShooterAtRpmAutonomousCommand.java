// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This command revs the shooter to a specified RPM. Because this command is to be
 * implemented in an autonomous program, it will end when the shooter motors reach their target
 * velocity but it will NOT stop the shooter motors unless it is interrupted.
 */
public class RevShooterAtRpmAutonomousCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  double targetRpm;
  double commandStartTimeMillis;
  double maxCommandRunTimeMillis = 5000;

  /** Creates a new RevShooterAtRpmAutonomousCommand. */
  public RevShooterAtRpmAutonomousCommand(double targetRpm, ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.targetRpm = targetRpm;
    commandStartTimeMillis = Timer.getMatchTime();
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setFlywheelTargetRpm(targetRpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      shooterSubsystem.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    This command will be considered finished in two different scenarios:
    1. The flywheel reaches its target velocity
    2. The command runtime surpasses the maximum runtime
    */
    return Timer.getMatchTime() > commandStartTimeMillis+maxCommandRunTimeMillis || shooterSubsystem.isFlywheelAtTargetVelocity();
  }
}
