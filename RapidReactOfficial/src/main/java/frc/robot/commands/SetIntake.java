// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntake extends CommandBase {

  IntakeSubsystem intakeSubsystem;
  boolean intakeIn;
  boolean finished = false;

  /** Creates a new SetIntake. */
  public SetIntake(boolean intakeIn, IntakeSubsystem intakeSubsystem) {
    this.intakeIn = intakeIn;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeIn) { 
      intakeSubsystem.intakeIn();
    } else {
      intakeSubsystem.intakeOut();
    }
    // finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
