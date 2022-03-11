// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndexCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.IndexSubsystem;

public class RunIndexToShootAutonomousCommand extends CommandBase {
  IndexSubsystem indexSubsystem;
  double maxCommandRunTimeSeconds = 3;
  int timesBeamBroken = 0;
  boolean wasBeamBrokenPrevious = false;
  int timesBeamNeedsToBeBroken = 2;
  Timer timer;

  /** Creates a new RunVerticalToShootCommand. */
  public RunIndexToShootAutonomousCommand(IndexSubsystem indexSubsystem) {
    this.indexSubsystem = indexSubsystem;
    timer = new Timer();
    addRequirements(indexSubsystem);
  }

  /** Creates a new RunVerticalToShootCommand. */
  public RunIndexToShootAutonomousCommand(int numberOfBallsToShoot, IndexSubsystem indexSubsystem) {
    this.indexSubsystem = indexSubsystem;
    timesBeamNeedsToBeBroken = numberOfBallsToShoot;
    timer = new Timer();
    addRequirements(indexSubsystem);
  }

  /** Creates a new RunVerticalToShootCommand. */
  public RunIndexToShootAutonomousCommand(double maxTimeSeconds, int numberOfBallsToShoot, IndexSubsystem indexSubsystem) {
    this.indexSubsystem = indexSubsystem;
    timesBeamNeedsToBeBroken = numberOfBallsToShoot;
    addRequirements(indexSubsystem);
    timer = new Timer();
    maxCommandRunTimeSeconds = maxTimeSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    indexSubsystem.setVerticalVoltage(ConstantsValues.verticalShootVolts);
    indexSubsystem.setRampVoltage(ConstantsValues.rampJogVolts);
    indexSubsystem.setTransferVoltage(ConstantsValues.transferJogVolts);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(indexSubsystem.isShooterSensorTriggered()) {
      if(!wasBeamBrokenPrevious) {
        timesBeamBroken++;
        wasBeamBrokenPrevious = true;
      }
    } else {
      wasBeamBrokenPrevious = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexSubsystem.stopAll();
    wasBeamBrokenPrevious = false;
    timesBeamBroken = 0;
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    This command should be considered finished in two scenarios:
    1. The beam break sensor has been broken once for each ball we expected to shoot
    2. The run time of this command has passed the max allowed run time
    */
    return timesBeamBroken >= timesBeamNeedsToBeBroken || timer.get() > maxCommandRunTimeSeconds;
  }
}
