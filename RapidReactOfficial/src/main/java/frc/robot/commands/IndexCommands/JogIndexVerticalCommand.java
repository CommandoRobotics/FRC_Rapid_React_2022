// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndexCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.IndexSubsystem;

public class JogIndexVerticalCommand extends CommandBase {

  IndexSubsystem indexSubsystem;

  /** Creates a new JogIndexVerticalCommand. */
  public JogIndexVerticalCommand(IndexSubsystem indexSubsystem) {
    this.indexSubsystem = indexSubsystem;
    addRequirements(indexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexSubsystem.setVerticalVoltage(ConstantsValues.verticalJogVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexSubsystem.stopVertical();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
