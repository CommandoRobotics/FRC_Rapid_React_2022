// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConstantsField;
import frc.robot.constants.ConstantsPorts;
import frc.robot.constants.ConstantsValues;

public class ShooterSubsystem extends SubsystemBase {

  // Declare different Limelight modes
  private final int LED_MODE_DEFAULT = 0;
  private final int LED_MODE_OFF = 1;
  private final int LED_MODE_BLINK = 2;
  private final int LED_MODE_ON = 3;
  private final int CAM_MODE_VISION_PROCESSOR = 0;
  private final int CAM_MODE_DRIVER_CAMERA = 1;
  private final int SNAPSHOT_MODE_DISABLED = 0;
  private final int SNAPSHOT_MODE_ENABLED = 1;

  // Motor controllers
  CANSparkMax flywheelLeader, flywheelFollowerOne, flywheelFollowerTwo, kickwheel;

  // Encoders
  RelativeEncoder flywheelEncoder, kickwheelEncoder;

  // Limelight
  NetworkTable limelight;

  // PID Controllers
  SparkMaxPIDController flywheelPidController, kickwheelPidController;

  // Feedforward
  SimpleMotorFeedforward flywheelFF, kickwheelFF;

  double previousP = ConstantsValues.flywheelP;

  public ShooterSubsystem() {
    // Instantiate the shooter motor controllers
    flywheelLeader = new CANSparkMax(ConstantsPorts.flywheelLeaderId, MotorType.kBrushless);
    flywheelFollowerOne = new CANSparkMax(ConstantsPorts.flywheelFollowerOneId, MotorType.kBrushless);
    flywheelFollowerTwo = new CANSparkMax(ConstantsPorts.flywheelFollowerTwoId, MotorType.kBrushless);
    kickwheel = new CANSparkMax(ConstantsPorts.kickwheelId, MotorType.kBrushless);

    // Reset the sparks
    flywheelLeader.restoreFactoryDefaults();
    flywheelFollowerOne.restoreFactoryDefaults();
    flywheelFollowerTwo.restoreFactoryDefaults();
    kickwheel.restoreFactoryDefaults();

    // Set the motor controllers to coast mode
    flywheelLeader.setIdleMode(IdleMode.kCoast);
    flywheelFollowerOne.setIdleMode(IdleMode.kCoast);
    flywheelFollowerTwo.setIdleMode(IdleMode.kCoast);
    kickwheel.setIdleMode(IdleMode.kCoast);

    // Set motor controller inversions
    //TODO set the proper motor inversions
    flywheelLeader.setInverted(false);
    // Other bottom flywheel inversions are set in the section below
    kickwheel.setInverted(false);

    // Set all bottom flywheel motor controllers to follow the main.
    // The boolean value represents whether the motor controller that is following
    // should be inverted relative to the main motor controller.
    flywheelFollowerOne.follow(flywheelLeader, false);
    flywheelFollowerTwo.follow(flywheelLeader, false);

    // Instantiate the encoders
    flywheelEncoder = flywheelLeader.getEncoder();
    kickwheelEncoder = kickwheel.getEncoder();

    // Instantiate the PID controllers
    flywheelPidController = flywheelLeader.getPIDController();
    kickwheelPidController = kickwheel.getPIDController();

    // Set PID controller values
    flywheelPidController.setP(ConstantsValues.flywheelP);
    flywheelPidController.setI(ConstantsValues.flywheelI);
    flywheelPidController.setD(ConstantsValues.flywheelD);
    flywheelPidController.setIZone(ConstantsValues.flywheelIZone);
    flywheelPidController.setFF(ConstantsValues.flywheelFF);
    flywheelPidController.setOutputRange(ConstantsValues.flywheelMinOutput, ConstantsValues.flywheelMaxOutput);
    kickwheelPidController.setP(ConstantsValues.kickwheelP);
    kickwheelPidController.setI(ConstantsValues.kickwheelI);
    kickwheelPidController.setD(ConstantsValues.kickwheelD);
    kickwheelPidController.setIZone(ConstantsValues.kickwheelIZone);
    kickwheelPidController.setFF(ConstantsValues.kickwheelFF);
    kickwheelPidController.setOutputRange(ConstantsValues.kickwheelMinOutput, ConstantsValues.kickwheelMaxOutput);

    // Instantiate motor feedforwards
    flywheelFF = new SimpleMotorFeedforward(ConstantsValues.flywheelKs, ConstantsValues.flywheelKv, ConstantsValues.flywheelKa);
    kickwheelFF = new SimpleMotorFeedforward(ConstantsValues.kickwheelKs, ConstantsValues.kickwheelKv, ConstantsValues.kickwheelKa);

    // Instantiate the limelight
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /**
   * Force the LED on the Limelight to be enabled
   */
  public void enableLimelightLed() {
    limelight.getEntry("ledMode").setNumber(LED_MODE_ON);
  }

  /**
   * Force the LED on the Limelight to be disabled
   */
  public void disableLimelightLed() {
    limelight.getEntry("ledMode").setNumber(LED_MODE_OFF);
  }

  /**
   * Set the LED on the Limelight to the default value in the current pipeline
   */
  public void limelightLedDefault() {
    limelight.getEntry("ledMode").setNumber(LED_MODE_DEFAULT);
  }

  /**
   * Enable vision processing on the Limelight
   */
  public void enableLimelightVisionProcessing() {
    limelight.getEntry("camMode").setNumber(CAM_MODE_VISION_PROCESSOR);
  }

  /**
   * Disable vision processing on the Limelight.
   * This increases exposure to let the camera feed function as a normal camera for the drivers.
   */
  public void disableLimelightVisionProcessing() {
    limelight.getEntry("camMode").setNumber(CAM_MODE_DRIVER_CAMERA);
  }

  /**
   * Set the Limelight's pipeline
   * @param pipeline An integer between and including 0 and 9 representing the new pipeline
   */
  public void setLimelightPipeline(int pipeline) {
    if(pipeline < 0 || pipeline > 9) {
      System.out.println("setLimelightPipeline ran into a number out of range");
      return;
    }
    limelight.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Enable taking snapshots on the Limelight.
   * This will take two snapshots per second.
   */
  public void enableLimelightSnapshot() {
    limelight.getEntry("snapshot").setNumber(SNAPSHOT_MODE_ENABLED);
  }

  /**
   * Disable taking snapshots on the Limelight.
   */
  public void disableLimelightSnapshot() {
    limelight.getEntry("snapshot").setNumber(SNAPSHOT_MODE_DISABLED);
  }

  /**
   * See if the Limelight currently sees any targets.
   * @return Whether the Limelight current sees any targets.
   */
  public boolean isTargetSeen() {
    return limelight.getEntry("tv").getDouble(0) > 0;
  }

  /**
   * Get the Limelight pipeline's latency contribution in ms.
   * Note: This does add 11ms extra to account for image capture latency
   * @return The Limelight pipeline's latency contribution in ms.
   */
  public double getLimelightLatency() {
    return limelight.getEntry("tl").getDouble(0)+11;
  }

  /**
   * Get the x offset of the target seen by the Limelight.
   * Note: This offset exists on a plane where the origin (or (0,0)) is the upper left corner.
   * Values to the right or down from this origin are positive in the x and y directions respectively.
   * Values can not be left or up relative to the origin of this plane.
   * @return The x offset of the target as seen by the limelight in pixels.
   */
  public double getLimelightXOffset() {
    double tx = limelight.getEntry("tx").getDouble(0);
    SmartDashboard.putNumber("tx", tx);
    return tx;
  }

  /**
   * Get the x offset of the target seen by the Limelight.
   * Note: This offset exists on a plane where the origin (or (0,0)) is the upper left corner.
   * Values to the right or down from this origin are positive in the x and y directions respectively.
   * Values can not be left or up relative to the origin of this plane.
   * @return The x offset of the target as seen by the limelight in pixels.
   */
  public double getLimelightYOffset() {
    return limelight.getEntry("ty").getDouble(0);
  }

  /**
   * Get the horizontal distance to the hub.
   * @return The horizontal distance to the hub in meters. 
   * Note: This value will be -1 in no target is visible.
   */
  public double getHorizontalDistanceToHub() {
    if(isTargetSeen()) {
      double changeInHeightMeters = ConstantsField.highHubRimHeightMeters-ConstantsValues.shooterHeightMeters;
      double totalAngle = getVerticalAngleToTarget();
      return changeInHeightMeters/Math.tan(Math.toRadians(totalAngle));
    } else {
      return -1;
    }
  }

  /**
   * Get the vertical angle to the target
   * @return The vertical angle to the target in degrees.
   */
  public double getVerticalAngleToTarget() {
    return getLimelightYOffset()+ConstantsValues.limelightMountingAngle;
  }

  /**
   * Set the voltage of the flywheel
   * @param volts
   */
  public void setFlywheelVoltage(double volts) {
    flywheelLeader.setVoltage(volts);
  }

  /**
   * Set the voltage of the kickwheel
   * @param volts
   */
  public void setKickwheelVoltage(double volts) {
    kickwheel.setVoltage(volts);
  }

  /**
   * Set the speed of the flywheel
   * @param speed
   */
  public void setFlywheelSpeed(double speed) {
    flywheelLeader.set(speed);
  }

  /**
   * Set the speed of the kickwheel
   * @param speed
   */
  public void setKickwheelSpeed(double speed) {
    kickwheel.set(speed);
  }

  /**
   * Get the position of the flywheel
   * @return The position of the flywheel in ticks
   */
  public double getFlywheelPosition() {
    return flywheelEncoder.getPosition();
  }
/**
 * Get the position of the kickwheel
 * @return The position of the kickwheel in ticks
 */
  public double getKickwheelPosition() {
    return kickwheelEncoder.getPosition();
  }

  /**
   * Get the RPM of the flywheel
   * @return The RPM of the flywheel
   */
  public double getFlywheelRPM() {
    return flywheelEncoder.getVelocity();
  }

  /**
   * Get the RPM of the kickwheel
   * @return The RPM of the flywheel
   */
  public double getKickwheelVelocity() {
    return kickwheelEncoder.getVelocity();
  }

  /**
   * Stop the flywheel
   */
  public void stopFlywheel() {
    flywheelLeader.set(0);
  }

  /**
   * Stop the kickwheel
   */
  public void stopKickwheel() {
    kickwheel.set(0);
  }

  /**
   * Stop the flywheel and the kickwheel
   */
  public void stopAll() {
    stopFlywheel();
    stopKickwheel();
  }

  /**
   * Set the RPM of the flywheel.
   * @param target The target RPM of the flywheel.
   */
  public void setFlywheelTargetRPM(double targetRPM) {
    flywheelPidController.setReference(
      targetRPM, 
      ControlType.kVelocity, 
      0, 
      //TODO might need to be in m/s
      flywheelFF.calculate(targetRPM), 
      ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("flywheelFF", flywheelFF.calculate(targetRPM));
  }

  /**
   * Set the RPM of the flywheel.
   * @param target The target RPM
   */
  public void setKickwheelTargetRPM(double targetRPM) {
    kickwheelPidController.setReference(
      targetRPM, 
      ControlType.kVelocity, 
      0, 
      //TODO might need to be in m/s
      kickwheelFF.calculate(targetRPM), 
      ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("flywheelFF", flywheelFF.calculate(targetRPM));
  }

  //TODO add a method for the kickwheel similar to the method above

  @Override
  public void periodic() {

    // Get the current flywheel values from the Smart dashboard
    // This will allow us to adjust these values on the fly from the Smart Dashboard
    double currentFlywheelP = SmartDashboard.getNumber("flywheelP", 0);
    double currentKickwheelP = SmartDashboard.getNumber("kickwheelP", 0);

    // Check if the numbers on the Smart Dashboard have changed, and change our values if they have.
    if (SmartDashboard.getNumber("flywheelP", ConstantsValues.flywheelP) != previousP) {
      flywheelPidController.setP(currentFlywheelP);
    }
    if (SmartDashboard.getNumber("kickwheelP", ConstantsValues.kickwheelP) != previousP) {
      kickwheelPidController.setP(currentKickwheelP);
    }

    // Write the feedforward values to the Smartdash
    SmartDashboard.putNumber("flywheelFF", ConstantsValues.flywheelFF);
    SmartDashboard.putNumber("kickwheelFF", ConstantsValues.kickwheelFF);
  }
}
