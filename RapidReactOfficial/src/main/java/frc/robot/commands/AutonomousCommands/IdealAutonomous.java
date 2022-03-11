// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAimCommands.AutoAimAutonomousCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootAutonomousCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtAutoVelocityAutonomousCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtRpmAutonomousCommand;
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
  /** Creates a new DoubleShotTaxiAutonomous. */
  public IdealAutonomous(
  DriveSubsystem driveSubsystem, 
  ShooterSubsystem shooterSubsystem, 
  AutoAimSubsystem autoAimSubsystem,
  IndexSubsystem indexSubsystem, 
  IntakeSubsystem intakeSubsystem) 
  {
    addCommands(

    // Start running the shooter 
    new RevShooterAtAutoVelocityAutonomousCommand(shooterSubsystem),

    // Reset gyro
    new InstantCommand(driveSubsystem::resetGyro),

    // Extend the intake, start the intake, and drive at the same time
    // Extend the intake
    new InstantCommand(intakeSubsystem::extend),
    new PrintCommand("Finished extending Intake"),
    
    // Start the intake
    new IntakeCommand(intakeSubsystem, indexSubsystem),
    new PrintCommand("Finished starting intake"),

    // Wait to extend and start intake
    new WaitCommand(0.5),

    // Drive to the ball outside tarmac
    driveSubsystem.newCommandFromTrajectory(
    PathFetcher.fetchDoubleShot(0), 
    true, // This is the intial pose
    true // The robot should stop after this trajectory is finished
    ),
    new PrintCommand("Finished DoubleShot Path 1"),

    // Actually shoot
    new RunIndexToShootAutonomousCommand(indexSubsystem),
    new PrintCommand("Finished running the index to shoot"),

    // Stop index and intake
    new InstantCommand(() -> indexSubsystem.stopAll(), indexSubsystem),
    new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
    new PrintCommand("Finished stopping index, and intake"),
    
    new PrintCommand("Finished DoubleShotAuto Portion"),

    new IntakeCommand(intakeSubsystem, indexSubsystem),
    new PrintCommand("Finished starting intake and index"),

    // Drive to to other ball
    driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchIdeal(1),
      false,
      true
    ),

    // Stop intake and index
    new InstantCommand(() -> indexSubsystem.stopAll(), indexSubsystem),
    new InstantCommand(intakeSubsystem::stop, intakeSubsystem),

    // Shoot
    new RunIndexToShootAutonomousCommand(1, indexSubsystem),

    // Stop index
    new InstantCommand(indexSubsystem::stopAll),

    // Start intake
    new IntakeCommand(intakeSubsystem, indexSubsystem),

    // Drive to last two balls
    driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchIdeal(2),
      false,
      true
    ),

    // Wait for human player to give us a ball
    new WaitCommand(0.9),

    // Drive to our shooting spot
    driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchIdeal(3),
      false,
      true
    ),

    // Stop the intake and index
    new InstantCommand(intakeSubsystem::stop),
    new InstantCommand(indexSubsystem::stopAll),

    // Run the index to shoot
    new RunIndexToShootAutonomousCommand(2, indexSubsystem)



    
    );

  }
}
