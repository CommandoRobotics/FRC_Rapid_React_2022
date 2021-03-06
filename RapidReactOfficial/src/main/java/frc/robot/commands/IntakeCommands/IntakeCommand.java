// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

  IntakeSubsystem intakeSubsystem;
  IndexSubsystem indexSubsystem;

  boolean isFinished = false;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexSubsystem = indexSubsystem;
    addRequirements(intakeSubsystem);
    addRequirements(indexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setPower(ConstantsValues.intakePower);
    indexSubsystem.setRampVoltage(ConstantsValues.rampIntakeVolts);
    indexSubsystem.setTransferVoltage(ConstantsValues.transferIntakeVolts);
    isFinished = true;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
