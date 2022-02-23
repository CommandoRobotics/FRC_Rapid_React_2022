// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Triggers.TriggerDash;
import frc.robot.commands.AimDrivetrainUsingVisionCommand;
import frc.robot.commands.DriveFieldCentricCommand;
import frc.robot.commands.DriveNotFieldCentricCommand;
import frc.robot.commands.DriveWithFieldCentricToggleCommand;
import frc.robot.commands.ExpelAllCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JogIndexRampCommand;
import frc.robot.commands.JogIndexRampReverseCommand;
import frc.robot.commands.JogIndexVerticalCommand;
import frc.robot.commands.JogIndexVerticalReverseCommand;
import frc.robot.commands.RevShooterAtAutoVelocityCommand;
import frc.robot.commands.RevShooterAtManualVelocityCommand;
import frc.robot.commands.RunVerticalToShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // Define controllers
  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);

  // Define alt triggers
  Trigger operatorAlt = new Trigger(() -> driverController.getLeftBumper());

  // Define subsystems
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ClimberSubsystem climberSubsystem = new ClimberSubsystem(); 
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IndexSubsystem indexSubsystem = new IndexSubsystem();
  AutoAimSubsystem autoAimSubsystem = new AutoAimSubsystem();

  // Network Tables
  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  NetworkTable commandoDashNT;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Network Tables Instantiation
    commandoDashNT = ntInst.getTable("CommandoDash");
    
    // Set any default commands
    // Driver sticks: drive
    driveSubsystem.setDefaultCommand(new DriveWithFieldCentricToggleCommand(driveSubsystem, 
    () -> driverController.getLeftY(),
    () -> -driverController.getLeftX(), 
    () -> -driverController.getRightX()));

    configureButtonBindings();
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
    /*
      DRIVER CONTROLLER
    */

    // Right trigger - Intake
    new Trigger(() -> driverController.getRightTriggerAxis() > 0.1)
    .whileActiveOnce(new IntakeCommand(intakeSubsystem, indexSubsystem));

    // Left trigger - Eject
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1)
    .whileActiveContinuous(intakeSubsystem::intakeOut);

    // Right bumper - Enable auto aim (drive based auto aim)
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
    .whileActiveOnce(new AimDrivetrainUsingVisionCommand(
      () -> driverController.getLeftY(), 
      () -> -driverController.getLeftX(), 
      () -> -driverController.getRightX(), 
      driveSubsystem, 
      autoAimSubsystem)
      );

    // Left bumper - Toggle intake lifter
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
    .whenActive(intakeSubsystem::toggleLifter);

    //TODO A - Hound Cargo

    // Back button - Expell all
    new JoystickButton(driverController, XboxController.Button.kBack.value)
    .whileActiveOnce(new ExpelAllCommand(intakeSubsystem, indexSubsystem, shooterSubsystem));

    // Start - Reset field centric driving
    new JoystickButton(driverController, XboxController.Button.kStart.value)
    .whenActive(driveSubsystem::resetGyro);

    // D-pad up - Toggle between field-centric and non field-centric drive
    new Trigger(() -> driverController.getPOV() == 0)
      .whenActive(driveSubsystem::toggleFieldCentric);

    /*
      OPERATOR CONTROLLER
    */
    // Left trigger and NOT a - Set shootervelocity to manually selected velocity
    new Trigger(() -> operatorController.getAButton())
    .negate()
    .and(new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.1))
    .whileActiveOnce(new RevShooterAtManualVelocityCommand(shooterSubsystem));

    // Left trigger and a - Set shooter velocity automatically based on Limelight
    new Trigger(() -> operatorController.getAButton())
    .and(new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.1))
    .whileActiveOnce(new RevShooterAtAutoVelocityCommand(shooterSubsystem));

    // Right bumper - Cycle manual shooter velocity
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
    .whenActive(shooterSubsystem::cycleManualVelocity);

    // Right trigger - Run vertical index to effectively shoot
    new Trigger(() -> (operatorController.getRightTriggerAxis() > 0.1))
    .whileActiveOnce(new RunVerticalToShootCommand(indexSubsystem));

    // Y and not alt - Jog index vertical
    operatorAlt.negate()
    .and(new JoystickButton(operatorController, XboxController.Button.kB.value))
    .whileActiveOnce(new JogIndexVerticalCommand(indexSubsystem));

    // Y and alt - Joy index vertical in reverse
    operatorAlt
    .and(new JoystickButton(operatorController, XboxController.Button.kY.value))
    .whileActiveOnce(new JogIndexVerticalReverseCommand(indexSubsystem));

    // B and alt - Jog index ramp in reverse
    operatorAlt
    .and(new JoystickButton(operatorController, XboxController.Button.kY.value))
    .whileActiveOnce(new JogIndexRampReverseCommand(indexSubsystem));

    // Start button - Expell all
    new JoystickButton(operatorController, XboxController.Button.kStart.value)
    .whileActiveOnce(new ExpelAllCommand(intakeSubsystem, indexSubsystem, shooterSubsystem));
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

}
