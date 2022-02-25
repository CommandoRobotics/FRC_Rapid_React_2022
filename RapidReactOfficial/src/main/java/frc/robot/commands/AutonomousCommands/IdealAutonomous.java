// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import javax.tools.StandardJavaFileManager.PathFactory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAimAutonomousCommand;
import frc.robot.commands.RunIndexToShootCommand;
import frc.robot.commands.SetShooterToVelocityCommand;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.PathFetcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IdealAutonomous extends SequentialCommandGroup {

  /** Creates a new IdealAutonomous. */
  public IdealAutonomous(
    DriveSubsystem driveSubsystem, 
    ShooterSubsystem shooterSubsystem, 
    IntakeSubsystem intakeSubsystem,
    IndexSubsystem indexSubsystem,
    AutoAimSubsystem autoAimSubsystem
    ) 
    {
    addCommands(

    // Aim towards the hub
    new AutoAimAutonomousCommand(driveSubsystem, autoAimSubsystem)
    // Set the shooter velocity
    .alongWith(new SetShooterToVelocityCommand(2000, shooterSubsystem))
    // Run the index to shoot
    .andThen(new RunIndexToShootCommand(indexSubsystem)),
    // Wait to give the robot time to shoot
    new WaitCommand(3),

    // Stop index and shooter
    new InstantCommand(indexSubsystem::stopAll, indexSubsystem),
    new InstantCommand(shooterSubsystem::stop, shooterSubsystem),

    // Extend intake
    new InstantCommand(intakeSubsystem::extend, intakeSubsystem),

    // Start intake
    new InstantCommand(intakeSubsystem::intakeIn, intakeSubsystem)
    // Drive to the first ball
    .alongWith(
      driveSubsystem.newCommandFromTrajectory(
        PathFetcher.fetchIdeal(0), 
        true, // This is the initial pose
        false // The robot should not stop once this path is complete
        )
    ),

    // Drive to the second ball
    driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchIdeal(1), 
      false, // This is not the initial pose
      true // The robot should stop at the end of this path
      ),

    // Stop intake
    new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
    
    // Aim towards the hub
    new AutoAimAutonomousCommand(driveSubsystem, autoAimSubsystem)
    // Set the shooter velocity
    .alongWith(new SetShooterToVelocityCommand(3000, shooterSubsystem))
    // Run the index to shoot
    .andThen(new RunIndexToShootCommand(indexSubsystem)),
    // Wait to give the robot time to shoot
    new WaitCommand(3),

    // Stop index and shooter
    new InstantCommand(indexSubsystem::stopAll, indexSubsystem),
    new InstantCommand(shooterSubsystem::stop, shooterSubsystem),
    
    // Start intake
    new InstantCommand(intakeSubsystem::intakeIn, intakeSubsystem),

    // Drive to two balls by terminal
    driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchIdeal(2), 
      false, // This is not the initial pose
      false // The robot should not stop after this path
    ),

    // Stop the intake
    new InstantCommand(intakeSubsystem::stop, intakeSubsystem),

    // Drive to better shooting position
    driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchIdeal(3),
      false, // This is not the initial pose
      true // The robot should stop after this path
    ),

    // Aim towards the hub
    new AutoAimAutonomousCommand(driveSubsystem, autoAimSubsystem)
    // Rev shooter to velocity
    .alongWith(new SetShooterToVelocityCommand(3500, shooterSubsystem))
    // Run index to shoot
    .andThen(new RunIndexToShootCommand(indexSubsystem)),
    // Wait a few seconds to shoot
    new WaitCommand(3),

    // Stop everything
    new InstantCommand(shooterSubsystem::stop, shooterSubsystem)
    .alongWith(new InstantCommand(indexSubsystem::stopAll, indexSubsystem))
    .alongWith(new InstantCommand(driveSubsystem::stop, driveSubsystem))
      
    );
  }
}
