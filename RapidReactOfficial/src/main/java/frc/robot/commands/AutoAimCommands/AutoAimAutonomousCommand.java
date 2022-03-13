// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAimCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAimAutonomousCommand extends CommandBase {

  DriveSubsystem driveSubsystem;
  AutoAimSubsystem autoAimSubsystem;
  Timer timer;
  boolean isFinished = false;
  double commandMaxRunTimeSeconds = 0.75;
  int noTargetIterations = 0;

  /** Creates a new AutoAimAutonomousCommand. */
  public AutoAimAutonomousCommand(DriveSubsystem driveSubsystem, AutoAimSubsystem autoAimSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.autoAimSubsystem = autoAimSubsystem;
    timer = new Timer();
    addRequirements(driveSubsystem, autoAimSubsystem);
  }

  /** Creates a new AutoAimAutonomousCommand. */
  public AutoAimAutonomousCommand(double maxTime, DriveSubsystem driveSubsystem, AutoAimSubsystem autoAimSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.autoAimSubsystem = autoAimSubsystem;
    commandMaxRunTimeSeconds = maxTime;
    timer = new Timer();
    addRequirements(driveSubsystem, autoAimSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoAimSubsystem.enableLimelightLed();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autoAimSubsystem.enableLimelightLed();
    double limelightXOffset = autoAimSubsystem.getLimelightXOffset();
    double panOutput = 0;
    if(autoAimSubsystem.isTargetSeen()) {
      // Calculate pan output
      panOutput = autoAimSubsystem.calculatePanOutput(limelightXOffset);

      // Decide if we need to end
      isFinished = panOutput == 0;

      noTargetIterations = 0;

    } else {
      noTargetIterations++;
    }

    // Drive the robot using the given pan output
    driveSubsystem.driveMecanum(0, 0, -panOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoAimSubsystem.disableLimelightLed();
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished || timer.get() > commandMaxRunTimeSeconds || noTargetIterations >= ConstantsValues.panPidMaxIterations;
  }
}
