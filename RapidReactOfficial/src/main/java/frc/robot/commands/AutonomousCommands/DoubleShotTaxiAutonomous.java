// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        new InstantCommand(intakeSubsystem::extend, intakeSubsystem),
        // Start the intake
        new IntakeCommand(intakeSubsystem, indexSubsystem)
      ),

      // Drive to the ball outside tarmac
      driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchDoubleShot(0), 
      true, // This is the intial pose
      false // The robot should not stop after this trajectory is finished
      )
    ),

    // Drive while waiting for a bit, stopping the intake, and bringing the intake in
    new ParallelCommandGroup(
      // Drive to the shooter position
      driveSubsystem.newCommandFromTrajectory(
        PathFetcher.fetchDoubleShot(1),
        false, // This is not the intial pose 
        true // The robot will stop after this trajectory is finished
      ),
      // Wait for a bit, stop the intake, bring the intake in, and rev the shooter
      new SequentialCommandGroup(
        // Wait
        new WaitCommand(1),
        // Stop the intake
        new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
        // Bring the intake back in
        new InstantCommand(intakeSubsystem::retract, intakeSubsystem),
        // Start reving the shooter
        new InstantCommand(()-> shooterSubsystem.setFlywheelTargetRpm(2000), shooterSubsystem)
      )
    ),

    // Auto aim
    new AutoAimAutonomousCommand(driveSubsystem, autoAimSubsystem),
    // Actually shoot
    new RunIndexToShootCommand(indexSubsystem),
    // Wait 3 seconds (for shooting)
    new WaitCommand(3),
    // Stop shooter and index
    new InstantCommand(() -> shooterSubsystem.stop(), shooterSubsystem),
    new InstantCommand(() -> indexSubsystem.stopAll(), indexSubsystem)

    );
  }
}
