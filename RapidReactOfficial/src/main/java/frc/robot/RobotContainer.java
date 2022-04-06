// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ConstantsValues;
import frc.robot.Triggers.TriggerPOV;
import frc.robot.Triggers.TriggerPOV.POVDirection;
import frc.robot.commands.AutoAimCommands.AutoAimCommand;
import frc.robot.commands.AutonomousCommands.DoubleShotAutonomous;
import frc.robot.commands.AutonomousCommands.IdealAutonomous;
import frc.robot.commands.AutonomousCommands.TaxiAutonomous;
import frc.robot.commands.DriveCommands.DriveWithFieldCentricToggleCommand;
import frc.robot.commands.IndexCommands.JogIndexRampCommand;
import frc.robot.commands.IndexCommands.JogIndexRampReverseCommand;
import frc.robot.commands.IndexCommands.JogIndexVerticalCommand;
import frc.robot.commands.IndexCommands.JogIndexVerticalReverseCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootCommand;
import frc.robot.commands.IndexCommands.RunIndexToShootWithBreakCommand;
import frc.robot.commands.IntakeCommands.HoundCargo;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.MiscellanousCommands.ExpelAllCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtAutoVelocityCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtManualVelocityCommand;
import frc.robot.commands.ShooterCommands.RevShooterAtRpmAutonomousCommand;
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
  Trigger driverAlt = new TriggerPOV(driverController, POVDirection.kDown);
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

    shooterSubsystem.disableLimelightLed();

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

    // Left trigger - Eject (intake)
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1)
    .whileActiveContinuous(intakeSubsystem::intakeOut)
    .whenInactive(intakeSubsystem::stop);

    // Left bumper - Toggle intake lifter
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
    .whenActive(intakeSubsystem::toggleExtend);

    // Right bumper - Run index to effectively shoot
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
    .whileActiveOnce(new RunIndexToShootCommand(indexSubsystem))
    .whenInactive(indexSubsystem::stopAll, indexSubsystem);

    // A and NOT alt - Auto aim and rev at auto velocity
    driverAlt.negate().and(
    new JoystickButton(driverController, XboxController.Button.kA.value))
      .whileActiveOnce(
        new AutoAimCommand(
        () -> -driverController.getLeftY(), 
        () -> driverController.getLeftX(), 
        () -> driverController.getRightX(), 
        driveSubsystem, 
        autoAimSubsystem)
        .alongWith(new RevShooterAtAutoVelocityCommand(shooterSubsystem))
      );

    // A and alt - Auto aim and rev at manual velocity
    driverAlt.and(
    new JoystickButton(driverController, XboxController.Button.kA.value))
        .whileActiveOnce(
          new AutoAimCommand(
         () -> -driverController.getLeftY(), 
         () -> driverController.getLeftX(), 
         () -> driverController.getRightX(), 
         driveSubsystem, 
        autoAimSubsystem)
        .alongWith(new RevShooterAtManualVelocityCommand(shooterSubsystem))
    );

    // B - Hound cargo
    new JoystickButton(driverController, XboxController.Button.kB.value)
      .whileActiveOnce(new HoundCargo(intakeSubsystem, driveSubsystem,       
                      () -> -driverController.getLeftY(), 
                      () -> driverController.getLeftX(), 
                      () -> driverController.getRightX()));

    // Y and NOT alt - Enable constant shooter reving
    driverAlt.negate().and(new JoystickButton(driverController, XboxController.Button.kY.value))
      .whenActive(new InstantCommand(() -> shooterSubsystem.setDefaultCommand(new RevShooterAtManualVelocityCommand(shooterSubsystem))));

    // Y and alt - Disable constant shooter reving
    driverAlt.and(new JoystickButton(driverController, XboxController.Button.kY.value)
      .whenActive(new InstantCommand(() -> shooterSubsystem.setDefaultCommand(new InstantCommand(shooterSubsystem::stop).perpetually()))));

    // X and NOT alt - Run shooter at manual velocity
    new JoystickButton(driverController, XboxController.Button.kX.value)
    .whenActive(new RevShooterAtRpmAutonomousCommand(5000, shooterSubsystem))
    .whenInactive(new InstantCommand(shooterSubsystem::stop));

    // // X and alt - Cycle manual shooter velocity
    // new JoystickButton(driverController, XboxController.Button.kX.value).and(driverAlt)
    // .whenActive(new InstantCommand(shooterSubsystem::cycleManualVelocity));

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
    // new JoystickButton(driverController, XboxController.Button.kX.value)
    // .whileActiveOnce(driveSubsystem.newCommandFromTrajectory(
    //   PathFetcher.fetchDifficultTestPath(0),
    //   true,
    //   true
    // ));

    // Y - Run drivetrain at a set speed
    new JoystickButton(driverController, XboxController.Button.kY.value)
    .whileActiveOnce(new InstantCommand(() -> driveSubsystem.driveMecanum(0.3, 0, 0)))
    .whenInactive(new InstantCommand(driveSubsystem::stop));

    /*
      OPERATOR CONTROLLER
    */
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

    // Left stick y and NOT right bumper - Winch up and down with limits
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value).negate().and(
    new Trigger(() -> (Math.abs(operatorController.getLeftY()) > 0)))
      .whileActiveContinuous(new InstantCommand(() -> climberSubsystem.setWinchVoltageFromController(operatorController.getLeftY())))
      .whenInactive(new InstantCommand(climberSubsystem::stopWinch));

    // Right stick y and NOT right bumper - Tilt forward and backward with limits
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value).negate().and(
    new Trigger(() -> (Math.abs(operatorController.getRightY()) > 0)))
      .whileActiveContinuous(new InstantCommand(() -> climberSubsystem.setTiltVoltageFromControllerSoftLimits(operatorController.getRightY())))
      .whenInactive(new InstantCommand(climberSubsystem::stopTilt));

    // Left stick y and right bumper - Winch up and down without limits
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value).and(
    new Trigger(() -> (Math.abs(operatorController.getLeftY()) > 0)))
      .whileActiveContinuous(new InstantCommand(() -> climberSubsystem.setWinchVoltageFromControllerNoLimits(operatorController.getLeftY())))
      .whenInactive(new InstantCommand(climberSubsystem::stopWinch));

    // Right stick y and right bumper - Tilt forward and backward with limits
    new JoystickButton(operatorController, XboxController.Button.kRightBumper.value).and(
      new Trigger(() -> (Math.abs(operatorController.getRightY()) > 0)))
        .whileActiveContinuous(new InstantCommand(() -> climberSubsystem.setTiltVoltageFromControllerNoLimits(operatorController.getRightY())))
        .whenInactive(new InstantCommand(climberSubsystem::stopTilt));

    // Dpad up - Reset winch encoder
    new TriggerPOV(operatorController, POVDirection.kUp)
    .whenActive(new InstantCommand(climberSubsystem::resetWinchEncoder));
    
    // Dpad down - Reset tilt encoder
    new TriggerPOV(operatorController, POVDirection.kDown)
    .whenActive(new InstantCommand(climberSubsystem::resetTiltEncoder));

  }

  /**
   * Disable the driver rumble
   */
  public void disableDriverRumble() {
    driverController.setRumble(RumbleType.kLeftRumble, 0);
    driverController.setRumble(RumbleType.kRightRumble, 0);
  }

  /**
   * Enable the driver rumble
   */
  public void enableDriverRumble() {
    driverController.setRumble(RumbleType.kLeftRumble, 1);
    driverController.setRumble(RumbleType.kRightRumble, 1);
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
