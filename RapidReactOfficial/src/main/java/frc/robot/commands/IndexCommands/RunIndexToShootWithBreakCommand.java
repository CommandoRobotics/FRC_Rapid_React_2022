// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndexCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IndexSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIndexToShootWithBreakCommand extends SequentialCommandGroup {

  /** Creates a new RunIndexToShootWithBreakCommand. */
  public RunIndexToShootWithBreakCommand(IndexSubsystem indexSubsystem) {
    SmartDashboard.putNumber("indexFirstShotDurationSec", 0.5);
    SmartDashboard.putNumber("indexPauseDurationSec", 0.25);
    addCommands(
      new RunIndexToShootCommand(indexSubsystem),
      new WaitCommand(SmartDashboard.getNumber("indexFirstShotDurationSec", 0.5)),
      new InstantCommand(indexSubsystem::stopAll, indexSubsystem),
      new WaitCommand(SmartDashboard.getNumber("indexPauseDurationSec", 0.25)),
      new RunIndexToShootCommand(indexSubsystem)
    );
  }
}
