// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Runs the intake in
 */
public class IntakeCommand extends CommandBase {

  IntakeSubsystem intakeSubsystem;
  IndexSubsystem indexSubsystem;

  public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexSubsystem = indexSubsystem;
    addRequirements(intakeSubsystem);
    addRequirements(indexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.intakeIn();
    indexSubsystem.setTransfer(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    indexSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
