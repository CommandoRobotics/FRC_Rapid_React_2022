// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IdealAutonomous extends SequentialCommandGroup {

  /** Creates a new IdealAutonomous. */
  public IdealAutonomous(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {

    addCommands();
  }
}
