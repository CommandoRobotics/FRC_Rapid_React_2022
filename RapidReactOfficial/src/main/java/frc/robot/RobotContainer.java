// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ShootAtRPMCommand;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Triggers.DashTrigger;
import frc.robot.commands.DriveFieldCentric;
import frc.robot.commands.DriveNotFieldCentric;
import frc.robot.commands.HoundCargo;
import frc.robot.commands.SetIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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

  // Define subsystems
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ClimberSubsystem climberSubsystem = new ClimberSubsystem(); 
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // Network Tables
  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  NetworkTable commandoDashNT;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Network Tables Instantiation
    commandoDashNT = ntInst.getTable("CommandoDash");
    
    // Set any default commands
    driveSubsystem.setDefaultCommand(new DriveFieldCentric(driveSubsystem, 
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
    new DashTrigger(commandoDashNT.getEntry("testData"), true)
      .whileActiveOnce(new HoundCargo(intakeSubsystem, driveSubsystem,
                                      () -> driverController.getLeftY(),
                                      () -> -driverController.getLeftX(), 
                                      () -> -driverController.getRightX()));

    new JoystickButton(driverController, XboxController.Button.kB.value)
      .whileActiveOnce(new HoundCargo(intakeSubsystem, driveSubsystem,
                                        () -> driverController.getLeftY(),
                                        () -> -driverController.getLeftX(), 
                                        () -> -driverController.getRightX()));
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
