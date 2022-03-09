// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAimAutonomousCommand;
import frc.robot.commands.IntakeCommand;
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

    // Extend the intake, start the intake, and drive at the same time
    new ParallelCommandGroup(
      // Extend the intake and THEN start it
      new SequentialCommandGroup(
        // Extend the intake
        new InstantCommand(intakeSubsystem::extend, intakeSubsystem).andThen(
          new PrintCommand("Finished Extending Intake")),
        // Start the intake
        new IntakeCommand(intakeSubsystem, indexSubsystem).andThen(
          new PrintCommand("Finished starting intake"))
      ),

      // Drive to the ball outside tarmac
      driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchDoubleShot(0), 
      true, // This is the intial pose
      false // The robot should not stop after this trajectory is finished
      ).andThen(
        new PrintCommand("Finished DoubleShot Path 1"))
    ),

    // Drive while waiting for a bit, stopping the intake, and bringing the intake in
    new ParallelCommandGroup(
      // Drive to the shooter position
      driveSubsystem.newCommandFromTrajectory(
        PathFetcher.fetchDoubleShot(1),
        false, // This is not the intial pose 
        true // The robot will stop after this trajectory is finished
      ).andThen(
        new PrintCommand("Finished DoubleShot Path 2")),
      // Wait for a bit, stop the intake, bring the intake in, and rev the shooter
      new SequentialCommandGroup(
        // Wait
        new WaitCommand(1).andThen(
          new PrintCommand("Finished Waiting 1 second")),
        // Stop the intake
        new InstantCommand(intakeSubsystem::stop, intakeSubsystem).andThen(
          new PrintCommand("Finished stopping the intake")),
        // Bring the intake back in
        new InstantCommand(intakeSubsystem::retract, intakeSubsystem).andThen(
          new PrintCommand("Finished retracting the intake")),
        // Start reving the shooter
        new InstantCommand(()-> shooterSubsystem.setFlywheelTargetRpm(2000), shooterSubsystem).andThen(
          new PrintCommand("Finished reving the shooter"))
      )
    ),

    // Auto aim
    new AutoAimAutonomousCommand(driveSubsystem, autoAimSubsystem).andThen(
      new PrintCommand("Finished starting to AutoAim")),
    // Actually shoot
    new RunIndexToShootCommand(indexSubsystem).andThen(
      new PrintCommand("Finished running the index to shoot")),
    // Wait 3 seconds (for shooting)
    new WaitCommand(3).andThen(
      new PrintCommand("Finished waiting 3 seconds")),
    // Stop shooter and index
    new InstantCommand(() -> shooterSubsystem.stop(), shooterSubsystem),
    new InstantCommand(() -> indexSubsystem.stopAll(), indexSubsystem).andThen(
      new PrintCommand("Finished stopping shooter and index")),
    new PrintCommand("Finished DoubleShotAuto"));
  }
}
