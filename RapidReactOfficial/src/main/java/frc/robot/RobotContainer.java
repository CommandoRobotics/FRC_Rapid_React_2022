// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.commands.ShootAtRPMCommand;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ConstantsValues;
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
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem(driveSubsystem);
  ClimberSubsystem climberSubsystem = new ClimberSubsystem(); 
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // Network Tables
  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  NetworkTable commandoDashNT;

  PathData field2dTest = new PathData("Field2d Test", false);
  Command field2dTestCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Network Tables Instantiation
    commandoDashNT = ntInst.getTable("CommandoDash");
    
    // Set any default commands
    driveSubsystem.setDefaultCommand(new DriveFieldCentric(driveSubsystem, 
    () -> driverController.getLeftY(),
    () -> -driverController.getLeftX(), 
    () -> -driverController.getRightX()));

    loadPathPlannerTrajectories(field2dTest);
    field2dTestCommand = driveSubsystem.createCommandFromPlannerTrajectory(field2dTest.trajectory, true, false);

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
      .whenActive(field2dTestCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  /**
   * This class simply allows us to store the PathPlanner name and the 
   * trajectory associated with it in the same place (and possibly the 
   * command created from it too - not used currently). It also allow you
   * to specify specifc max velocity and accelerations if needed.
   */
  public class PathData {
    public String PathName;
    public PathPlannerTrajectory trajectory;
    public boolean reversed;
    public double maxVel = ConstantsValues.driveMaxVel;
    public double maxAccel = ConstantsValues.driveMaxAcc;
    
    public PathData(String pathWeaverJSON, boolean reversed) 
      {PathName = pathWeaverJSON; this.reversed = reversed;}

    public PathData(String pathWeaverJSON, boolean reversed, 
                    double maxVel, double maxAccel) 
      {PathName = pathWeaverJSON; this.reversed = reversed;
       this.maxVel = maxVel; this.maxAccel = maxAccel;}
  }

  /**
   * Takes the given PathDatas, generates the trajectory associated with the
   * JSONName they have, and then sets the PathData's trajectory to that trajectory
   * @param pathWeaverData PathData class to load trajectories to
   */
  private void loadPathPlannerTrajectories(PathData... pathData) {
    for (PathData pData:pathData) {
      pData.trajectory = PathPlanner.loadPath(
        pData.PathName,
        pData.maxVel,
        pData.maxAccel,
        pData.reversed);
    }
  }

}
