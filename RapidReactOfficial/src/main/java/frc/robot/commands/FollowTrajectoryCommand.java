// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectoryCommand extends CommandBase {

  private final Timer m_timer = new Timer();
  private final PathPlannerTrajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SimpleMotorFeedforward m_feedforward;
  private final MecanumDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Supplier<Rotation2d> m_desiredRotation;
  private final double m_maxWheelVelocityMetersPerSecond;
  private final PIDController m_frontLeftController;
  private final PIDController m_rearLeftController;
  private final PIDController m_frontRightController;
  private final PIDController m_rearRightController;
  private final Supplier<MecanumDriveWheelSpeeds> m_currentWheelSpeeds;
  private final Consumer<MecanumDriveMotorVoltages> m_outputDriveVoltages;
  private final Consumer<MecanumDriveWheelSpeeds> m_outputWheelSpeeds;
  private final boolean usingCustomdriveRotationInput;
  private final boolean outputWheelSpeeds;
  private MecanumDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  /** 
   * Simplified Command for following a trajectory using a mecanum drive. This command is also based
   * around PathPlanner trajectories and will by default use the PathPlannerTrajectory's rotation to
   * set the rotation of the robot during the trajectory following. If you do not want to follow the trajectories
   * rotation, please use the other contructor that accepts a custom rotation input. It will automatically
   * set all the wheels to PIDF calculated volatages.<p>
   * 
   * </p><b>Note:</b> this command does not stop the drivetrain upon completion or interuption to allow the command 
   * to continue into other trajectory followers
   * 
   * @param trajectory the PathPlanner trajectory to follow
   * @param driveSubsystem the driveSubsystem to use/require
   */
  public FollowTrajectoryCommand(PathPlannerTrajectory trajectory, DriveSubsystem driveSubsystem) {
    PIDController xController = new PIDController(ConstantsValues.driveXP, ConstantsValues.driveXI, ConstantsValues.driveXD);
    PIDController yController = new PIDController(ConstantsValues.driveYP, ConstantsValues.driveYI, ConstantsValues.driveYD);
    ProfiledPIDController rProfiledPIDController =
     new ProfiledPIDController(
        ConstantsValues.driveRotationP, 
        ConstantsValues.driveRotationI, 
        ConstantsValues.driveRotationD, 
        new TrapezoidProfile.Constraints(
          ConstantsValues.rotationMaxVelocityMetersPerSec, 
          ConstantsValues.rotationMaxAccelerationMetersPerSecPerSec));
    m_trajectory = trajectory;
    m_pose = driveSubsystem::getPose; //Supplier to get the current Pose2d of the robot
    m_feedforward = ConstantsValues.mecanumFeedForward;
    m_kinematics = ConstantsValues.mecanumDriveKinematics;
    m_controller =
        new HolonomicDriveController(xController, yController, rProfiledPIDController);
    m_maxWheelVelocityMetersPerSecond = ConstantsValues.maxWheelVelocityMetersPerSecond;
    m_desiredRotation = null;
    m_frontLeftController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
    m_rearLeftController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
    m_frontRightController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
    m_rearRightController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
    m_currentWheelSpeeds = driveSubsystem::getWheelSpeeds; //Supplier to get the current MecanumWheelSpeeds
    m_outputDriveVoltages = driveSubsystem::setVoltages; //Consumer that will give a MecanumDriveMotorVoltages containing the volts to set each motor to
    m_outputWheelSpeeds = null; //Comsumer that will give a MecanumDriveWheelSpeeds contains the velocites each motor should be at
    usingCustomdriveRotationInput = false;
    outputWheelSpeeds = false;
    SmartDashboard.putData("X PID Controller", xController);
    SmartDashboard.putData("Y PID Controller", yController);
    SmartDashboard.putData("Rotation PID Controller", rProfiledPIDController);
    SmartDashboard.putData("Front Left Trajectory PID", m_frontLeftController);
    SmartDashboard.putData("Front Right Trajectory PID", m_frontRightController);
    SmartDashboard.putData("Rear Left Trajectory PID", m_rearLeftController);
    SmartDashboard.putData("Rear Right Trajectory PID", m_rearRightController);
    //TODO add your specific subsystem type
    addRequirements(driveSubsystem);
  }

  /** 
   * Simplified Command for following a trajectory using a mecanum drive. This constructor will force the Command
   * to use a custom rotation supplier for rotation target during the trajectory following. It will automatically
   * set all the wheels to PIDF calculated volatages.<p>
   * 
   * </p><b>Note:</b> this command does not stop the drivetrain upon completion or interuption to allow the command 
   * to continue into other trajectory followers
   * 
   * @param trajectory the PathPlanner trajectory to follow
   * @param desiredRotation a supplier that will supply the desired rotation while trajectory following
   * @param driveSubsystem the driveSubsystem to use/require
   */
  public FollowTrajectoryCommand(PathPlannerTrajectory trajectory, Supplier<Rotation2d> desiredRotation,
  DriveSubsystem driveSubsystem) {
      PIDController xController = new PIDController(ConstantsValues.driveXP, ConstantsValues.driveXI, ConstantsValues.driveXD);
      PIDController yController = new PIDController(ConstantsValues.driveYP, ConstantsValues.driveYI, ConstantsValues.driveYD);
      ProfiledPIDController rProfiledPIDController =
       new ProfiledPIDController(
          ConstantsValues.driveRotationP, 
          ConstantsValues.driveRotationI, 
          ConstantsValues.driveRotationD, 
          new TrapezoidProfile.Constraints(
            ConstantsValues.rotationMaxVelocityMetersPerSec, 
            ConstantsValues.rotationMaxAccelerationMetersPerSecPerSec));
      m_trajectory = trajectory;
      m_pose = driveSubsystem::getPose; //Supplier to get the current Pose2d of the robot
      m_feedforward = ConstantsValues.mecanumFeedForward;
      m_kinematics = ConstantsValues.mecanumDriveKinematics;
      m_controller =
          new HolonomicDriveController(xController, yController, rProfiledPIDController);
      m_maxWheelVelocityMetersPerSecond = ConstantsValues.maxWheelVelocityMetersPerSecond;
      m_desiredRotation = desiredRotation;
      m_frontLeftController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
      m_rearLeftController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
      m_frontRightController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
      m_rearRightController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
      m_currentWheelSpeeds = driveSubsystem::getWheelSpeeds; //Supplier to get the current MecanumWheelSpeeds
      m_outputDriveVoltages = driveSubsystem::setVoltages; //Consumer that will give a MecanumDriveMotorVoltages containing the volts to set each motor to
      m_outputWheelSpeeds = null; //Comsumer that will give a MecanumDriveWheelSpeeds contains the velocites each motor should be at
      usingCustomdriveRotationInput = true;
      outputWheelSpeeds = false;
      SmartDashboard.putData("X PID Controller", xController);
      SmartDashboard.putData("Y PID Controller", yController);
      SmartDashboard.putData("Rotation PID Controller", rProfiledPIDController);
      SmartDashboard.putData("Front Left Trajectory PID", m_frontLeftController);
      SmartDashboard.putData("Front Right Trajectory PID", m_frontRightController);
      SmartDashboard.putData("Rear Left Trajectory PID", m_rearLeftController);
      SmartDashboard.putData("Rear Right Trajectory PID", m_rearRightController);
      //TODO add your specific subsystem type
      addRequirements(driveSubsystem);
  }

  /** 
   * Simplified Command for following a trajectory using a mecanum drive. This constructor will force the Command
   * to use a custom rotation supplier for rotation target during the trajectory following. It will automatically
   * set all the wheels to PIDF calculated volatages.<p>
   * 
   * </p>This version will ouput the target wheel velocities (m/s) instead of running the PIDF loops itself 
   * to figure out the wheel voltages.<p>
   * 
   * </p><b>Note:</b> this command does not stop the drivetrain upon completion or interuption to allow the command 
   * to continue into other trajectory followers
   * 
   * @param trajectory the PathPlanner trajectory to follow
   * @param desiredRotation a supplier that will supply the desired rotation while trajectory following
   * @param outputWheelSpeeds whether or not to output WheelSpeeds in m/s instead of PIDF calculated voltages
   * @param driveSubsystem the driveSubsystem to use/require
   */
  public FollowTrajectoryCommand(PathPlannerTrajectory trajectory, Supplier<Rotation2d> desiredRotation,
    boolean outputWheelSpeeds, DriveSubsystem driveSubsystem) {
    PIDController xController = new PIDController(ConstantsValues.driveXP, ConstantsValues.driveXI, ConstantsValues.driveXD);
    PIDController yController = new PIDController(ConstantsValues.driveYP, ConstantsValues.driveYI, ConstantsValues.driveYD);
    ProfiledPIDController rProfiledPIDController =
    new ProfiledPIDController(
        ConstantsValues.driveRotationP, 
        ConstantsValues.driveRotationI, 
        ConstantsValues.driveRotationD, 
        new TrapezoidProfile.Constraints(
          ConstantsValues.rotationMaxVelocityMetersPerSec, 
          ConstantsValues.rotationMaxAccelerationMetersPerSecPerSec));
    m_trajectory = trajectory;
    m_pose = driveSubsystem::getPose; //Supplier to get the current Pose2d of the robot
    m_feedforward = ConstantsValues.mecanumFeedForward;
    m_kinematics = ConstantsValues.mecanumDriveKinematics;
    m_controller =
        new HolonomicDriveController(xController, yController, rProfiledPIDController);
    m_maxWheelVelocityMetersPerSecond = ConstantsValues.maxWheelVelocityMetersPerSecond;
    m_desiredRotation = desiredRotation;
    m_currentWheelSpeeds = driveSubsystem::getWheelSpeeds; //Supplier to get the current MecanumWheelSpeeds
    usingCustomdriveRotationInput = true;
    if (outputWheelSpeeds) {
      m_frontLeftController = null;
      m_rearLeftController = null;
      m_frontRightController = null;
      m_rearRightController = null;
      this.outputWheelSpeeds = true;
      m_outputDriveVoltages = null; //Consumer that will give a MecanumDriveMotorVoltages containing the volts to set each motor to
      m_outputWheelSpeeds = driveSubsystem::setWheelSpeeds; //Comsumer that will give a MecanumDriveWheelSpeeds contains the velocites each motor should be at
    } else {
      m_frontLeftController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
      m_rearLeftController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
      m_frontRightController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
      m_rearRightController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
      SmartDashboard.putData("Front Left Trajectory PID", m_frontLeftController);
      SmartDashboard.putData("Front Right Trajectory PID", m_frontRightController);
      SmartDashboard.putData("Rear Left Trajectory PID", m_rearLeftController);
      SmartDashboard.putData("Rear Right Trajectory PID", m_rearRightController);
      this.outputWheelSpeeds = false;
      m_outputDriveVoltages = driveSubsystem::setVoltages; //Consumer that will give a MecanumDriveMotorVoltages containing the volts to set each motor to
      m_outputWheelSpeeds = null; //Comsumer that will give a MecanumDriveWheelSpeeds contains the velocites each motor should be at
    }
    SmartDashboard.putData("X PID Controller", xController);
    SmartDashboard.putData("Y PID Controller", yController);
    SmartDashboard.putData("Rotation PID Controller", rProfiledPIDController);
    //TODO add your specific subsystem type
    addRequirements(driveSubsystem);
  }

  /** 
   * Simplified Command for following a trajectory using a mecanum drive. This command is also based
   * around PathPlanner trajectories and will by default use the PathPlannerTrajectory's rotation to
   * set the rotation of the robot during the trajectory following. If you do not want to follow the trajectories
   * rotation, please use the other contructor that accepts a custom rotation input.<p>
   * 
   * </p>This version will ouput the target wheel velocities (m/s) instead of running the PIDF loops itself 
   * to figure out the wheel voltages.<p>
   * 
   * </p><b>Note:</b> this command does not stop the drivetrain upon completion or interuption to allow the command 
   * to continue into other trajectory followers
   * 
   * @param trajectory the PathPlanner trajectory to follow
   * @param outputWheelSpeeds whether or not to output WheelSpeeds in m/s instead of PIDF calculated voltages
   * @param driveSubsystem the driveSubsystem to use/require
   */
  public FollowTrajectoryCommand(PathPlannerTrajectory trajectory, boolean outputWheelSpeeds,
  DriveSubsystem driveSubsystem) {
      PIDController xController = new PIDController(ConstantsValues.driveXP, ConstantsValues.driveXI, ConstantsValues.driveXD);
      PIDController yController = new PIDController(ConstantsValues.driveYP, ConstantsValues.driveYI, ConstantsValues.driveYD);
      ProfiledPIDController rProfiledPIDController =
      new ProfiledPIDController(
          ConstantsValues.driveRotationP, 
          ConstantsValues.driveRotationI, 
          ConstantsValues.driveRotationD, 
          new TrapezoidProfile.Constraints(
            ConstantsValues.rotationMaxVelocityMetersPerSec, 
            ConstantsValues.rotationMaxAccelerationMetersPerSecPerSec));
      m_trajectory = trajectory;
      m_pose = driveSubsystem::getPose; //Supplier to get the current Pose2d of the robot
      m_feedforward = ConstantsValues.mecanumFeedForward;
      m_kinematics = ConstantsValues.mecanumDriveKinematics;
      m_controller =
          new HolonomicDriveController(xController, yController, rProfiledPIDController);
      m_maxWheelVelocityMetersPerSecond = ConstantsValues.maxWheelVelocityMetersPerSecond;
      m_desiredRotation = null;
      m_currentWheelSpeeds = driveSubsystem::getWheelSpeeds; //Supplier to get the current MecanumWheelSpeeds
      usingCustomdriveRotationInput = false;
      if (outputWheelSpeeds) {
        m_frontLeftController = null;
        m_rearLeftController = null;
        m_frontRightController = null;
        m_rearRightController = null;
        this.outputWheelSpeeds = true;
        m_outputDriveVoltages = null; //Consumer that will give a MecanumDriveMotorVoltages containing the volts to set each motor to
        m_outputWheelSpeeds = driveSubsystem::setWheelSpeeds; //Comsumer that will give a MecanumDriveWheelSpeeds contains the velocites each motor should be at
      } else {
        m_frontLeftController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
        m_rearLeftController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
        m_frontRightController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
        m_rearRightController = new PIDController(ConstantsValues.driveWheelP, ConstantsValues.driveWheelI, ConstantsValues.driveWheelD);
        SmartDashboard.putData("Front Left Trajectory PID", m_frontLeftController);
        SmartDashboard.putData("Front Right Trajectory PID", m_frontRightController);
        SmartDashboard.putData("Rear Left Trajectory PID", m_rearLeftController);
        SmartDashboard.putData("Rear Right Trajectory PID", m_rearRightController);
        this.outputWheelSpeeds = false;
        m_outputDriveVoltages = driveSubsystem::setVoltages; //Consumer that will give a MecanumDriveMotorVoltages containing the volts to set each motor to
        m_outputWheelSpeeds = null; //Comsumer that will give a MecanumDriveWheelSpeeds contains the velocites each motor should be at
      }
      SmartDashboard.putData("X PID Controller", xController);
      SmartDashboard.putData("Y PID Controller", yController);
      SmartDashboard.putData("Rotation PID Controller", rProfiledPIDController);
      //TODO add your specific subsystem type
      addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var initialState = m_trajectory.sample(0);

    var initialXVelocity = initialState.velocityMetersPerSecond * initialState.poseMeters.getRotation().getCos();
    var initialYVelocity = initialState.velocityMetersPerSecond * initialState.poseMeters.getRotation().getSin();

    m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialXVelocity, initialYVelocity, 0.0));

    //Post the current trajectory to the Field2d object
    Field2d field = (Field2d) SmartDashboard.getData("Field");
    field.getObject("Current Robot Trajectory").setTrajectory(m_trajectory);

    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    //Get the desired state of the robot (cast to a PathPlannerState to get rotation)
    var desiredState = (PathPlannerState) m_trajectory.sample(curTime);
    ChassisSpeeds targetChassisSpeeds;
    //Use the Holonomic Controller to get the ChassisSpeeds required to get to the next state based on the previous state
    if (usingCustomdriveRotationInput) {
      SmartDashboard.putBoolean("Using PathPlanner Rotation", false);
      targetChassisSpeeds =
      m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    } else {
      SmartDashboard.putBoolean("Using PathPlanner Rotation", true);
      targetChassisSpeeds =
      m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
    }

    //Change ChassisSpeeds to WheelSpeeds (m/s) using mecanum kinematics
    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(targetChassisSpeeds);

    targetWheelSpeeds.desaturate(m_maxWheelVelocityMetersPerSecond);

    //Set the PID setpoints for each wheel
    var frontLeftSpeedSetpoint = targetWheelSpeeds.frontLeftMetersPerSecond;
    var rearLeftSpeedSetpoint = targetWheelSpeeds.rearLeftMetersPerSecond;
    var frontRightSpeedSetpoint = targetWheelSpeeds.frontRightMetersPerSecond;
    var rearRightSpeedSetpoint = targetWheelSpeeds.rearRightMetersPerSecond;

    if (outputWheelSpeeds) {
      m_outputWheelSpeeds.accept(targetWheelSpeeds);
    } else {
      double frontLeftOutput;
      double rearLeftOutput;
      double frontRightOutput;
      double rearRightOutput;

      //Calculate the feedforward for each wheel based on the given feedforward for the chassis
      final double frontLeftFeedforward =
          m_feedforward.calculate(
              frontLeftSpeedSetpoint,
              (frontLeftSpeedSetpoint - m_prevSpeeds.frontLeftMetersPerSecond) / dt);
      final double rearLeftFeedforward =
          m_feedforward.calculate(
              rearLeftSpeedSetpoint,
              (rearLeftSpeedSetpoint - m_prevSpeeds.rearLeftMetersPerSecond) / dt);
      final double frontRightFeedforward =
          m_feedforward.calculate(
              frontRightSpeedSetpoint,
              (frontRightSpeedSetpoint - m_prevSpeeds.frontRightMetersPerSecond) / dt);
      final double rearRightFeedforward =
          m_feedforward.calculate(
              rearRightSpeedSetpoint,
              (rearRightSpeedSetpoint - m_prevSpeeds.rearRightMetersPerSecond) / dt);

      //Get the outputs of each PID controllers for each wheel based on the setpoint and feedforward
      frontLeftOutput =
          frontLeftFeedforward
              + m_frontLeftController.calculate(
                  m_currentWheelSpeeds.get().frontLeftMetersPerSecond, frontLeftSpeedSetpoint);
      rearLeftOutput =
          rearLeftFeedforward
              + m_rearLeftController.calculate(
                  m_currentWheelSpeeds.get().rearLeftMetersPerSecond, rearLeftSpeedSetpoint);
      frontRightOutput =
          frontRightFeedforward
              + m_frontRightController.calculate(
                  m_currentWheelSpeeds.get().frontRightMetersPerSecond, frontRightSpeedSetpoint);
      rearRightOutput =
          rearRightFeedforward
              + m_rearRightController.calculate(
                  m_currentWheelSpeeds.get().rearRightMetersPerSecond, rearRightSpeedSetpoint);

      //Feed the output voltages of the PID loops to the drive train as a MecanumDriveMotorVoltages
      m_outputDriveVoltages.accept(
          new MecanumDriveMotorVoltages(
              frontLeftOutput, frontRightOutput, rearLeftOutput, rearRightOutput));
    }

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Get rid of the Trajectory
    NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Field").getEntry("Current Robot Trajectory")
      .forceSetNumberArray(new Number[]{0,0,0});
    //Stop the timer
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
