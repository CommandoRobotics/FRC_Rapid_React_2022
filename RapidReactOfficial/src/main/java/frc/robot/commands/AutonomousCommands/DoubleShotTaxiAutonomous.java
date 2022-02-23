// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ConstantsValues;
import frc.robot.commands.AimDrivetrainUsingVisionCommand;
import frc.robot.commands.AutoAimAutonomousCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RevShooterAtManualVelocityCommand;
import frc.robot.commands.RunIndexToShootCommand;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.PathFetcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleShotTaxiAutonomous extends SequentialCommandGroup {
  /** Creates a new DoubleShotTaxiAutonomous. */
  public DoubleShotTaxiAutonomous(
  DriveSubsystem driveSubsystem, 
  ShooterSubsystem shooterSubsystem, 
  AutoAimSubsystem autoAimSubsystem,
  IndexSubsystem indexSubsystem, 
  IntakeSubsystem intakeSubsystem) 
  {
    addCommands(
      // Extend the intake
      new InstantCommand(intakeSubsystem::lowerLifter, intakeSubsystem),

      // Start the intake
      new IntakeCommand(intakeSubsystem, indexSubsystem),

      // Drive to ball outside tarmac
      new FollowTrajectoryCommand(
      PathFetcher.fetchDoubleShot(0), driveSubsystem),

      // Wait half a second
      new WaitCommand(0.5),

      // Stop intake
      new InstantCommand(intakeSubsystem::stop, intakeSubsystem),

      // Bring the intake back in
      new InstantCommand(intakeSubsystem::raiseLifter, intakeSubsystem),

      // Drive to shooting zone
      new FollowTrajectoryCommand(
      PathFetcher.fetchDoubleShot(1), driveSubsystem),

      // Start reving shooter
      new InstantCommand(() -> shooterSubsystem.setFlywheelTargetRpm(2000), shooterSubsystem),

      // Auto aim
      new AutoAimAutonomousCommand(driveSubsystem, autoAimSubsystem),

      // Actually shoot
      new RunIndexToShootCommand(indexSubsystem),

      // Wait 3 seconds (for shooting)
      new WaitCommand(3),

      // Stop shooter and index
      new InstantCommand(() -> shooterSubsystem.stop()),
      new InstantCommand(() -> indexSubsystem.stopAll())

    );
  }
}
