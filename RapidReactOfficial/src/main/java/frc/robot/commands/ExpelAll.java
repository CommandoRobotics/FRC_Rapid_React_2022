// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ExpelAll extends CommandBase {

  IntakeSubsystem intakeSubsystem;
  IndexSubsystem indexSubsystem;
  ShooterSubsystem shooterSubsystem;

  /** Creates a new ExpelAll. */
  public ExpelAll(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexSubsystem = indexSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(intakeSubsystem);
    addRequirements(indexSubsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.intakeOut();
    indexSubsystem.setVertical(ConstantsValues.verticalExpelPower);
    indexSubsystem.setRamp(ConstantsValues.rampExpelPower);
    indexSubsystem.setTransfer(ConstantsValues.transferExpelPower);
    shooterSubsystem.setTopVoltage(ConstantsValues.topFlywheelExpelVolts);
    shooterSubsystem.setBottomVoltage(ConstantsValues.bottomFlywheelExpelVolts);
    shooterSubsystem.setKickwheelVoltage(ConstantsValues.kickwheelExpelVolts);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    indexSubsystem.stopAll();
    shooterSubsystem.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
