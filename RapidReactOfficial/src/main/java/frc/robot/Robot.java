// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private NetworkTableInstance ntInst;
  private NetworkTable commandoDashNT;
  private String selectedAuto = "Taxi - Default"; //TODO add default clause
  private Alliance previousAlliance = Alliance.Blue;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    ntInst = NetworkTableInstance.getDefault();
    commandoDashNT = ntInst.getTable("CommandoDash");

    m_robotContainer = new RobotContainer(ntInst);

    //Tell CommandoDash we're currently disabled
    commandoDashNT.getSubTable("AllianceAndModeData").getEntry("robotMode").setNumber(32);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //Update auto chooser
    selectedAuto = commandoDashNT.getEntry("autoSelection").getString("Taxi - Default"); //TODO add default clause
    commandoDashNT.getEntry("rioAutoSelection").setString(selectedAuto);

    //Update Robot Color
    Alliance currAlliance = DriverStation.getAlliance();
    if (currAlliance == Alliance.Red && previousAlliance != currAlliance) {
      commandoDashNT.getSubTable("AllianceAndModeData").getEntry("alliance").setNumber(1);
    } else if (currAlliance == Alliance.Blue && previousAlliance != currAlliance) {
      commandoDashNT.getSubTable("AllianceAndModeData").getEntry("alliance").setNumber(0);
    }
    previousAlliance = currAlliance;

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //Tell CommandoDash we're currently disabled
    commandoDashNT.getSubTable("AllianceAndModeData").getEntry("robotMode").setNumber(32);
  }

  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //Tell CommandoDash we're in auto
    commandoDashNT.getSubTable("AllianceAndModeData").getEntry("robotMode").setNumber(35);

    selectedAuto = commandoDashNT.getEntry("autoSelection").getString("Taxi - Default"); //TODO add default clause
    commandoDashNT.getEntry("rioAutoSelection").setString(selectedAuto);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(selectedAuto);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //Tell CommandoDash we're in teleop
    commandoDashNT.getSubTable("AllianceAndModeData").getEntry("robotMode").setNumber(33);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    //Tell CommandoDash we're in test
    commandoDashNT.getSubTable("AllianceAndModeData").getEntry("robotMode").setNumber(37);
    
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
