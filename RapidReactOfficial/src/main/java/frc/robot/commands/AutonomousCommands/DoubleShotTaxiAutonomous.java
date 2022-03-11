// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootAutonomousCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtAutoVelocityAutonomousCommand;
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
public class DoubleShotTaxiAutonomous extends SequentialCommandGroup {
  /** Creates a new DoubleShotTaxiAutonomous. */
  public DoubleShotTaxiAutonomous(
  DriveSubsystem driveSubsystem, 
  ShooterSubsystem shooterSubsystem, 
  AutoAimSubsystem autoAimSubsystem,
  IndexSubsystem indexSubsystem, 
  IntakeSubsystem intakeSubsystem, 
  ClimberSubsystem climberSubsystem) 
  {
    addCommands(

    // Reset gyro
    new InstantCommand(driveSubsystem::resetGyro),
    new PrintCommand("Finished resetting the gyro"),

    // Retract the climber (just in case)
    new InstantCommand(climberSubsystem::midDown),
    new PrintCommand("Finished retracting the climber"),

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

    // Rev the shooter
    new RevShooterAtAutoVelocityAutonomousCommand(shooterSubsystem),
    new PrintCommand("Finished reving shooter"),

    new InstantCommand(autoAimSubsystem::enableLimelightSnapshot),

    // Actually shoot
    new RunIndexToShootAutonomousCommand(indexSubsystem),
    new PrintCommand("Finished running the index to shoot"),

    // Stop shooter and index
    new InstantCommand(() -> shooterSubsystem.stop(), shooterSubsystem),
    new InstantCommand(() -> indexSubsystem.stopAll(), indexSubsystem),
    new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
    new PrintCommand("Finished stopping shooter and index"),

    // Pull the intake back in
    new InstantCommand(intakeSubsystem::retract)
    .andThen(new PrintCommand("Finished retracing intake")),
    
    new PrintCommand("Finished DoubleShotAuto"));
  }
}
