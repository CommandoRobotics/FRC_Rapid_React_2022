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
import frc.robot.commands.ShooterCommands.SetShooterToVelocityCommand;
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
      // Extend the intake, start the intake, and drive at the same time
      new ParallelCommandGroup(
        // Extend the intake and THEN start it
        new SequentialCommandGroup(
          // Extend the intake
          new InstantCommand(intakeSubsystem::extend, intakeSubsystem).andThen(
            new PrintCommand("Finished extending Intake")),
          // Start the intake
          new IntakeCommand(intakeSubsystem, indexSubsystem).andThen(
            new PrintCommand("Finished starting intake"))
        ),
        // Drive to the ball outside tarmac
        driveSubsystem.newCommandFromTrajectory(
        PathFetcher.fetchIdeal(0), 
        true, // This is the intial pose
        true // The robot should stop after this trajectory is finished
        ).andThen(
          new PrintCommand("Finished Ideal Path 1"))
      ),
      // Stop the index and intake
      new InstantCommand(indexSubsystem::stopAll),
      new InstantCommand(intakeSubsystem::stop),

      // Rev the shooter to the automatically determined velocity
      new RevShooterAtAutoVelocityAutonomousCommand(shooterSubsystem)
      .andThen(new PrintCommand("Finished reving shooter")),

      // Shoot two balls
      new RunIndexToShootAutonomousCommand(indexSubsystem)
      .andThen(new PrintCommand("Finished running index to shoot 2 balls")),

      // Stop shooter and index
      new InstantCommand(shooterSubsystem::stop)
      .andThen(new InstantCommand(indexSubsystem::stopAll))
      .andThen(new PrintCommand("Finished stopping the shooter and the index")),

      // Start driving to the next ball while running the intake
      new IntakeCommand(intakeSubsystem, indexSubsystem)
      .alongWith(
        driveSubsystem.newCommandFromTrajectory(
          PathFetcher.fetchIdeal(1),
          false,
          true
        )
      ),
      new PrintCommand("Finished Ideal Path 2"),

      // Stop the index and intake
      new InstantCommand(indexSubsystem::stopAll),
      new InstantCommand(intakeSubsystem::stop),
      new PrintCommand("Finished stopping index and intake"),

      // Rev the shooter ot the automatically determined velocity
      new RevShooterAtAutoVelocityAutonomousCommand(shooterSubsystem),
      new PrintCommand("Finished reving shooter to auto velocity"),

      // Run index to shoot one ball
      new RunIndexToShootAutonomousCommand(1, indexSubsystem),
      new PrintCommand("Finished shooting 1 ball"),


      // Stop shooter and index
      new InstantCommand(shooterSubsystem::stop)
      .andThen(new InstantCommand(indexSubsystem::stopAll))
      .andThen(new PrintCommand("Finished stopping the shooter and the index")),

      // Starting driving to the next ball while running the intake
      // Start driving to the next ball while running the intake
      new IntakeCommand(intakeSubsystem, indexSubsystem)
      .alongWith(
        driveSubsystem.newCommandFromTrajectory(
          PathFetcher.fetchIdeal(2),
          false,
          true
        )
      ),
      new PrintCommand("Finished Ideal Path 3"),

      // Retract the intake for protection.
      // Note: we keep running it here to make sure we fully intake the balls we gathered
      new InstantCommand(intakeSubsystem::retract),
      new PrintCommand("Finished retracting intake"),

      // Drive to our shooting position
      driveSubsystem.newCommandFromTrajectory(
          PathFetcher.fetchIdeal(3),
          false,
          true
      ),
      new PrintCommand("Finished Ideal Path 4"),

      // Stop intake and index
      new InstantCommand(indexSubsystem::stopAll),
      new InstantCommand(intakeSubsystem::stop),
      new PrintCommand("Finished stopping index and intake"),

      // Rev shooter to automatically determined velocity
      new RevShooterAtAutoVelocityAutonomousCommand(shooterSubsystem),
      new PrintCommand("Finished reving shooter to auto velocity"),

      // Shoot two balls
      new RunIndexToShootAutonomousCommand(indexSubsystem),
      new PrintCommand("Finished shooting 2 balls"),
    

      // Stop everything
      new InstantCommand(intakeSubsystem::stop),
      new InstantCommand(indexSubsystem::stopAll),
      new InstantCommand(shooterSubsystem::stop),
      new PrintCommand("Finished stopping all motors"),

      new PrintCommand("Finished IdealAuto")

    );
  }
}
