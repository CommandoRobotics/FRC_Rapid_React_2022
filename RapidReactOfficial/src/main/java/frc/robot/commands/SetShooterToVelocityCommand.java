// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A command that sets the shooter to a certain velocity. 
 * This command with automatically finish once the shooter reaches the target velocity.
 * This command does NOT stop the shooter once the command has completed, but it 
 * DOES stop the shooter if the command is interrupted.
 */
public class SetShooterToVelocityCommand extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  double targetRpm;
  int iterationsWhereTargetIsMet = 0;
  boolean isFinished = false;

  /** Creates a new SetShooterToVelocityCommand. */
  public SetShooterToVelocityCommand(double targetRpm, ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
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
    if(
    shooterSubsystem.getFlywheelVelocity() > targetRpm-ConstantsValues.flywheelAtVelocityDeadband
    &&
    shooterSubsystem.getFlywheelVelocity() < targetRpm+ConstantsValues.flywheelAtVelocityDeadband
    ) {
      iterationsWhereTargetIsMet++;
    } else {
      iterationsWhereTargetIsMet = 0;
    }
    isFinished = iterationsWhereTargetIsMet >= ConstantsValues.flywheelAtVelocityIterations;
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
    return isFinished;
  }
}
