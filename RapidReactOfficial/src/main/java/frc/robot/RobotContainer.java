// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Triggers.TriggerPOV;
import frc.robot.Triggers.TriggerPOV.POVDirection;
import frc.robot.commands.AutoAimCommands.AutoAimCommand;
import frc.robot.commands.AutonomousCommands.DoubleShotAutonomous;
import frc.robot.commands.AutonomousCommands.IdealAutonomous;
import frc.robot.commands.AutonomousCommands.TaxiAutonomous;
import frc.robot.commands.DriveCommands.DriveWithFieldCentricToggleCommand;
import frc.robot.commands.DriveCommands.FollowTrajectoryCommand;
import frc.robot.commands.IndexCommands.JogIndexRampCommand;
import frc.robot.commands.IndexCommands.JogIndexRampReverseCommand;
import frc.robot.commands.IndexCommands.JogIndexVerticalCommand;
import frc.robot.commands.IndexCommands.JogIndexVerticalReverseCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootAutoEndCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootAutoEndCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootWithBreakCommand;
import frc.robot.commands.IntakeCommands.HoundCargo;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.MiscellanousCommands.ExpelAllCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtAutoVelocityAutonomousCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtAutoVelocityCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtManualVelocityCommand;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.PathFetcher;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Network Tables
  NetworkTableInstance ntInst;
  NetworkTable commandoDashNT;
  
  // Define controllers
  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);

  // Define alt triggers
  Trigger operatorAlt = new TriggerPOV(operatorController, POVDirection.kLeft);

  // Define subsystems
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem(driveSubsystem);
  ClimberSubsystem climberSubsystem = new ClimberSubsystem(); 
  IndexSubsystem indexSubsystem = new IndexSubsystem();
  AutoAimSubsystem autoAimSubsystem = new AutoAimSubsystem();
  PowerSubsystem powerSubsystem = new PowerSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(NetworkTableInstance networkTableInst) {

    // Network Tables Instantiation
    ntInst = networkTableInst;
    commandoDashNT = ntInst.getTable("CommandoDash");

    DriverStation.silenceJoystickConnectionWarning(true);

    // Load all paths
    PathFetcher.loadAllPaths();
    
    // Set any default commands
    // Driver sticks: drive
    driveSubsystem.setDefaultCommand(new DriveWithFieldCentricToggleCommand(driveSubsystem, 
    () -> -driverController.getLeftY(),
    () -> driverController.getLeftX(), 
    () -> driverController.getRightX()));

    shooterSubsystem.enableLimelightLed();

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
    .whenActive(new IntakeCommand(intakeSubsystem, indexSubsystem))
    .whenInactive(intakeSubsystem::stop, intakeSubsystem)
    .whenInactive(indexSubsystem::stopAll, intakeSubsystem);

    // Left trigger - Eject
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1)
    .whileActiveContinuous(intakeSubsystem::intakeOut)
    .whenInactive(intakeSubsystem::stop);

    // Right bumper - Enable auto aim (drive based auto aim)
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
    .whileActiveOnce(new AutoAimCommand(
      () -> -driverController.getLeftY(), 
      () -> driverController.getLeftX(), 
      () -> driverController.getRightX(), 
      driveSubsystem, 
      autoAimSubsystem)
      );

    // Left bumper - Toggle intake lifter
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
    .whenActive(intakeSubsystem::toggleExtend);

    // A - Hound cargo
    new JoystickButton(driverController, XboxController.Button.kA.value)
      .whileActiveOnce(new HoundCargo(intakeSubsystem, driveSubsystem,       
                      () -> -driverController.getLeftY(), 
                      () -> driverController.getLeftX(), 
                      () -> driverController.getRightX()));

    // Back button - Expell all
    new JoystickButton(driverController, XboxController.Button.kBack.value)
    .whileActiveOnce(new ExpelAllCommand(intakeSubsystem, indexSubsystem, shooterSubsystem));

    // Start - Reset field centric driving
    new JoystickButton(driverController, XboxController.Button.kStart.value)
    .whenActive(driveSubsystem::resetGyro);

    // D-pad up - Toggle between field-centric and non field-centric drive
    new Trigger(() -> driverController.getPOV() == 0)
      .whenActive(driveSubsystem::toggleFieldCentric);

    // X - Start running the difficult test path
    new JoystickButton(driverController, XboxController.Button.kX.value)
    .whileActiveOnce(driveSubsystem.newCommandFromTrajectory(
      PathFetcher.fetchDifficultTestPath(0),
      true,
      true
    ));

    // Y - Run drivetrain at a set speed
    new JoystickButton(driverController, XboxController.Button.kY.value)
    .whileActiveOnce(new InstantCommand(() -> driveSubsystem.driveMecanum(0.3, 0, 0)))
    .whenInactive(new InstantCommand(driveSubsystem::stop));

    /*
      OPERATOR CONTROLLER
    */
    // Left trigger and a - Set shootervelocity to manually selected velocity
    new Trigger(() -> operatorController.getAButton())
    .and(new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.1))
    .whileActiveOnce(new RevShooterAtManualVelocityCommand(shooterSubsystem));

    // Left trigger and NOT a - Set shooter velocity automatically based on Limelight
    new Trigger(() -> operatorController.getAButton())
    .negate()
    .and(new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.1))
    .whileActiveOnce(new RevShooterAtAutoVelocityCommand(shooterSubsystem));

    // Right bumper - Cycle manual shooter velocity
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
    .whenActive(shooterSubsystem::cycleManualVelocity);

    // Right trigger and NOT alt - Run vertical index to effectively shoot
    operatorAlt.negate().and(
    new Trigger(() -> (operatorController.getRightTriggerAxis() > 0.1)))
    .whileActiveOnce(new RunIndexToShootCommand(indexSubsystem))
    .whenInactive(indexSubsystem::stopAll, indexSubsystem);

    // Right trigger and alt - Run vertical index to effectively shoot
    operatorAlt.and(
    new Trigger(() -> (operatorController.getRightTriggerAxis() > 0.1)))
    .whileActiveOnce(new RunIndexToShootCommand(indexSubsystem))
    .whenInactive(indexSubsystem::stopAll, indexSubsystem);

    // Y and not alt - Jog index vertical
    operatorAlt.negate()
    .and(new JoystickButton(operatorController, XboxController.Button.kY.value))
    .whileActiveOnce(new JogIndexVerticalCommand(indexSubsystem));

    // Y and alt - Joy index vertical in reverse
    operatorAlt
    .and(new JoystickButton(operatorController, XboxController.Button.kY.value))
    .whileActiveOnce(new JogIndexVerticalReverseCommand(indexSubsystem));

    // B and not alt - Jog index ramp
    operatorAlt.negate()
    .and(new JoystickButton(operatorController, XboxController.Button.kB.value))
    .whileActiveContinuous(new JogIndexRampCommand(indexSubsystem));

    // B and alt - Jog index ramp in reverse
    operatorAlt
    .and(new JoystickButton(operatorController, XboxController.Button.kB.value))
    .whileActiveOnce(new JogIndexRampReverseCommand(indexSubsystem));

    // Start button - Expell all
    new JoystickButton(operatorController, XboxController.Button.kStart.value)
    .whileActiveOnce(new ExpelAllCommand(intakeSubsystem, indexSubsystem, shooterSubsystem));

    // Left stick y - Winch up and down
    new Trigger(() -> (Math.abs(operatorController.getLeftY()) > 0))
      .whileActiveContinuous(new InstantCommand(() -> climberSubsystem.setWinch(operatorController.getLeftY())))
      .whenInactive(new InstantCommand(climberSubsystem::stopWinch));

    // Right stick y - Tilt forward and backward
    new Trigger(() -> (Math.abs(operatorController.getRightY()) > 0))
      .whileActiveContinuous(new InstantCommand(() -> climberSubsystem.setTiltNoLimits(operatorController.getRightY())))
      .whenInactive(new InstantCommand(climberSubsystem::stopTilt));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String autoSelected) {
    switch (autoSelected) {
      case "IdealAuto":
        return new IdealAutonomous(driveSubsystem, shooterSubsystem, autoAimSubsystem, indexSubsystem, intakeSubsystem, climberSubsystem);
      case "DoubleShot":
        return new DoubleShotAutonomous(driveSubsystem, shooterSubsystem, autoAimSubsystem, indexSubsystem, intakeSubsystem, climberSubsystem);
      case "Spare":
        return null; //TODO Add "Spare" command
      case "FullSend":
        return null; //TODO Add "FullSend" command
      case "Taxi":
        return new TaxiAutonomous(driveSubsystem, shooterSubsystem, autoAimSubsystem, indexSubsystem, intakeSubsystem, climberSubsystem);
      case "DoubleShot - Default":
        return new IdealAutonomous(driveSubsystem, shooterSubsystem, autoAimSubsystem, indexSubsystem, intakeSubsystem, climberSubsystem); //TODO Add "Taxi - Default" command
      default:
        return new DoubleShotAutonomous(driveSubsystem, shooterSubsystem, autoAimSubsystem, indexSubsystem, intakeSubsystem, climberSubsystem); //TODO Determine default command (or have null? tho I wouldn't recommend that)
    }
  }
}
