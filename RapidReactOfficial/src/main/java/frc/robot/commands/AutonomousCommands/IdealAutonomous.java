// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAimCommands.AutoAimAutonomousCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootAutoEndCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtAutoVelocityNoStopCommand;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
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

  RevShooterAtAutoVelocityNoStopCommand revCommandOne;
  RevShooterAtAutoVelocityNoStopCommand revCommandTwo;

  public IdealAutonomous(
  DriveSubsystem driveSubsystem, 
  ShooterSubsystem shooterSubsystem, 
  AutoAimSubsystem autoAimSubsystem,
  IndexSubsystem indexSubsystem, 
  IntakeSubsystem intakeSubsystem,
  ClimberSubsystem climberSubsystem) 
  {
    revCommandOne = new RevShooterAtAutoVelocityNoStopCommand(shooterSubsystem);
    revCommandTwo = new RevShooterAtAutoVelocityNoStopCommand(shooterSubsystem);

    addCommands(

    // Reset gyro
    new InstantCommand(driveSubsystem::resetGyro),
    new PrintCommand("Finished resetting gyro"),

    // Extend the intake
    new InstantCommand(intakeSubsystem::extend),
    new PrintCommand("Finished extending Intake"),
    
    // Start the intake
    new IntakeCommand(intakeSubsystem, indexSubsystem),
    new PrintCommand("Finished starting intake"),

    // Start running the shooter while waiting and then driving
    revCommandOne.raceWith(
      new SequentialCommandGroup(
        // Wait and then drive
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          driveSubsystem.newCommandFromTrajectory(
            PathFetcher.fetchDoubleShot(0),
            true,
            true
          )
        ),

        new PrintCommand("Finished DoubleShot Path 1"),

        // Stop intake
        new InstantCommand(intakeSubsystem::stop),
        new PrintCommand("Finished stopping the intake"),

        // Auto aim
        new AutoAimAutonomousCommand(0.75, driveSubsystem, autoAimSubsystem),
        new PrintCommand("Finished auto aim"),

        // Actually shoot
        new RunIndexToShootAutoEndCommand(1.25, 2, indexSubsystem),
        new PrintCommand("Finished running the index to shoot"),

        // Stop index and intake
        new InstantCommand(indexSubsystem::stopAll, indexSubsystem),
        new PrintCommand("Finished stopping the index"),
        
        new PrintCommand("Finished DoubleShotAuto Portion"),

        new IntakeCommand(intakeSubsystem, indexSubsystem),
        new PrintCommand("Finished intake command"),

        // Drive to other ball
        driveSubsystem.newCommandFromTrajectory(
          PathFetcher.fetchIdeal(1),
          false,
          true
        ),
        new PrintCommand("Finished Ideal Path 2"),

        // Stop intake and index
        new InstantCommand(() -> indexSubsystem.stopAll(), indexSubsystem),
        new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
        new PrintCommand("Finished stopping index and intake"),

        // Auto aim
        new AutoAimAutonomousCommand(0.75, driveSubsystem, autoAimSubsystem),
        new PrintCommand("Finished auto aim"),

        // Shoot
        new RunIndexToShootAutoEndCommand(0.75, 1, indexSubsystem),
        new PrintCommand("Finished shooting 2 balls"),

        // Stop index and shooter
        new InstantCommand(indexSubsystem::stopAll, indexSubsystem)  
      )
    ),

    new PrintCommand("Finished stoppign the index and the shooter"),
    
    // Start intake
    new IntakeCommand(intakeSubsystem, indexSubsystem),
    new PrintCommand("Start the intake"),

    // Drive to last two balls
    driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchIdeal(2),
      false,
      true
    ),
    new PrintCommand("Finished Ideal Path 3"),

    // Wait for human player to give us a ball
    new WaitCommand(0.9),
    new PrintCommand("Finished waiting for ball from human player"),

    // Drive to our shooting spot while reving the shooter
    revCommandTwo.raceWith(
      new SequentialCommandGroup(
        driveSubsystem.newCommandFromTrajectory(
        PathFetcher.fetchIdeal(3),
        false,
        true),

        new PrintCommand("Finished reving shooter and Ideal Path 4"),


        // Stop the intake and index
        new InstantCommand(intakeSubsystem::stop),
        new InstantCommand(indexSubsystem::stopAll),
        new PrintCommand("Finished stopping the intake and index"),

        // Auto aim
        new AutoAimAutonomousCommand(0.75, driveSubsystem, autoAimSubsystem),
        new PrintCommand("Finished auto aim"),

        // Run the index to shoot
        new RunIndexToShootAutoEndCommand(2, 2, indexSubsystem),
        new PrintCommand("Finished running index to shoot"),

        // Stop everything
        new InstantCommand(driveSubsystem::stop, driveSubsystem),
        new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
        new InstantCommand(indexSubsystem::stopAll, indexSubsystem),

        new InstantCommand(revCommandTwo::cancel),
        new PrintCommand("Finished stopping everthing")
      )

    ),
    // Retract intake
    new InstantCommand(intakeSubsystem::retract),
    new PrintCommand("Finished retracting intake"),

    new PrintCommand("Finished Ideal Auto")

    );

  }
}
