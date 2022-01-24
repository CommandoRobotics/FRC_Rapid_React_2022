// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  CANSparkMax topFlywheel, bottomFlywheelOne, bottomFlywheelTwo, bottomFlywheelThree, kickwheel;

  // Motor controller groups
  MotorControllerGroup bottomFlywheel;

  // Encoders
  RelativeEncoder topFlywheelEncoder, bottomFlywheelOneEncoder, bottomFlywheelTwoEncoder, 
  bottomFlywheelThreeEncoder, kickwheelEncoder;

  // Limelight
  NetworkTable limelight;
  NetworkTableEntry validTargets, xOfTarget, yOfTarget, areaOfTarget, 
  latency, currentPipeline, shortestSideLength, longestSideLength;

  public ShooterSubsystem() {
    // Instantiate the shooter motor controllers
    topFlywheel = new CANSparkMax(ConstantsPorts.topFlywheelId, MotorType.kBrushless);
    bottomFlywheelOne = new CANSparkMax(ConstantsPorts.bottomFlywheelOneId, MotorType.kBrushless);
    bottomFlywheelTwo = new CANSparkMax(ConstantsPorts.bottomFlywheelTwoId, MotorType.kBrushless);
    bottomFlywheelThree = new CANSparkMax(ConstantsPorts.bottomFlywheelThreeId, MotorType.kBrushless);
    kickwheel = new CANSparkMax(ConstantsPorts.kickwheelId, MotorType.kBrushless);

    // Set the motor controllers to coast mode
    topFlywheel.setIdleMode(IdleMode.kCoast);
    bottomFlywheelOne.setIdleMode(IdleMode.kCoast);
    bottomFlywheelTwo.setIdleMode(IdleMode.kCoast);
    bottomFlywheelThree.setIdleMode(IdleMode.kCoast);
    kickwheel.setIdleMode(IdleMode.kCoast);

    // Set motor controller inversions
    topFlywheel.setInverted(false);
    bottomFlywheelOne.setInverted(false);
    bottomFlywheelTwo.setInverted(false);
    bottomFlywheelThree.setInverted(false);
    kickwheel.setInverted(false);

    // Instantiate the encoders
    topFlywheelEncoder = topFlywheel.getEncoder();
    bottomFlywheelOneEncoder = bottomFlywheelOne.getEncoder();
    bottomFlywheelTwoEncoder = bottomFlywheelTwo.getEncoder();
    bottomFlywheelThreeEncoder = bottomFlywheelThree.getEncoder();
    kickwheelEncoder = kickwheel.getEncoder();

    // Set encoder inversions
    topFlywheelEncoder.setInverted(false);
    bottomFlywheelOneEncoder.setInverted(false);
    bottomFlywheelTwoEncoder.setInverted(false);
    bottomFlywheelThreeEncoder.setInverted(false);
    kickwheelEncoder.setInverted(false);

    // Set the velocity conversion factors
    topFlywheelEncoder.setVelocityConversionFactor(ConstantsValues.topFlywheelVelocityConversionFactor);
    bottomFlywheelOneEncoder.setVelocityConversionFactor(ConstantsValues.bottomFlywheelVelocityConversionFactor);
    bottomFlywheelTwoEncoder.setVelocityConversionFactor(ConstantsValues.bottomFlywheelVelocityConversionFactor);
    bottomFlywheelThreeEncoder.setVelocityConversionFactor(ConstantsValues.bottomFlywheelVelocityConversionFactor);
    kickwheelEncoder.setVelocityConversionFactor(ConstantsValues.kickwheelVelocityConversionFactor);


    // Instantiate the motor controller groups
    bottomFlywheel = new MotorControllerGroup(bottomFlywheelOne, bottomFlywheelTwo, bottomFlywheelThree);

    // Instantiate the limelight and its network table entries
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    validTargets = limelight.getEntry("tv");
    xOfTarget = limelight.getEntry("tx");
    yOfTarget = limelight.getEntry("ty");
    areaOfTarget = limelight.getEntry("ta");
    latency = limelight.getEntry("tl");
    currentPipeline = limelight.getEntry("getpipe");
    shortestSideLength = limelight.getEntry("tshort");
    longestSideLength = limelight.getEntry("tlong");
  }

  /**
   * Force the LED on the Limelight to be enabled
   */
  public void enableLed() {
    limelight.getEntry("ledMode").setNumber(LED_MODE_ON);
  }

  /**
   * Force the LED on the Limelight to be disabled
   */
  public void disableLed() {
    limelight.getEntry("ledMode").setNumber(LED_MODE_OFF);
  }

  /**
   * Set the LED on the Limelight to the default value in the current pipeline
   */
  public void ledDefault() {
    limelight.getEntry("ledMode").setNumber(LED_MODE_DEFAULT);
  }

  /**
   * Enable vision processing on the Limelight
   */
  public void enableVisionProcessing() {
    limelight.getEntry("camMode").setNumber(CAM_MODE_VISION_PROCESSOR);
  }

  /**
   * Disable vision processing on the Limelight.
   * This increases exposure to let the camera feed function as a normal camera for the drivers.
   */
  public void disableVisionProcessing() {
    limelight.getEntry("camMode").setNumber(CAM_MODE_DRIVER_CAMERA);
  }

  /**
   * Set the Limelight's pipeline
   * @param pipeline An integer between and including 0 and 9 representing the new pipeline
   */
  public void setPipeline(int pipeline) {
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
  public void enableSnapshot() {
    limelight.getEntry("snapshot").setNumber(SNAPSHOT_MODE_ENABLED);
  }

  /**
   * Disable taking snapshots on the Limelight.
   */
  public void disableSnapshot() {
    limelight.getEntry("snapshot").setNumber(SNAPSHOT_MODE_DISABLED);
  }

  /**
   * See if the Limelight currently sees any targets.
   * @return Whether the Limelight current sees any targets.
   */
  public boolean isTargetSeen() {
    return validTargets.getBoolean(false);
  }

  /**
   * Get a value representing the target's horizontal offset from the crosshair
   * @return A value representing the target's horizontal offset from the crosshair
   */
  public double getXOfTarget() {
    return xOfTarget.getDouble(0.0);
  }

  /**
   * Get a value representing the target's vertical offset from the crosshair
   * @return A value representing the target's vertical offset from the crosshair
   */
  public double getYOfTarget() {
    return yOfTarget.getDouble(0.0);
  }

  /**
   * Get a value representing the percent area of the target
   * @return A value representing the ratio of target area to screen area
   */
  public double getAreaOfTarget() {
    return areaOfTarget.getDouble(0.0);
  }

  /**
   * Get the latency of the Limelight, including image capture latency
   * @return The latency of the Limelight
   */
  public double getLimelightLatency() {
    // Add 11 due to the image capture latency
    return latency.getDouble(0.0)+11;
  }

  /**
   * Get the length, in pixels, of the shortest side of the fitted box
   * @return The length, in pixels, of the shortest side of the fitted box
   */
  public double getShortestSideLength() {
    return shortestSideLength.getDouble(0.0);
  }

  /**
   * Get the length, in pixels, of the longest side of the fitted box
   * @return The length, in pixels, of the longest side of the fitted box
   */
  public double getLongestSideLength() {
    return longestSideLength.getDouble(0.0);
  }

  /**
   * Set the voltage of the bottom flywheel
   * @param volts
   */
  public void setBottomFlywheelVoltage(double volts) {
    bottomFlywheel.setVoltage(volts);
  }

  /**
   * Set the voltage of the top flywheel
   * @param volts
   */
  public void setTopFlywheelVoltage(double volts) {
    topFlywheel.setVoltage(volts);
  }

  /**
   * Set the voltage of both the top and bottom flywheels
   * @param volts
   */
  public void setBothFlywheelsVoltage(double volts) {
    topFlywheel.setVoltage(volts);
    bottomFlywheel.setVoltage(volts);
  }

  /**
   * Set the voltage of the kickwheel
   * @param volts
   */
  public void setKickwheelVoltage(double volts) {
    kickwheel.setVoltage(volts);
  }

  /**
   * Set the speed of the bottom flywheel
   * @param speed
   */
  public void setBottomFlywheelSpeed(double speed) {
    bottomFlywheel.set(speed);
  }

  /**
   * Set the speed of the top flywheel
   * @param speed
   */
  public void setTopFlywheelSpeed(double speed) {
    topFlywheel.set(speed);
  }

  /**
   * Set the speed of the kickwheel
   * @param speed
   */
  public void setKickwheelSpeed(double speed) {
    kickwheel.set(speed);
  }

  /**
   * Set the speed of both the top and bottom flywheels
   * @param speed
   */
  public void setBothFlywheelsSpeed(double speed) {
    topFlywheel.set(speed);
    bottomFlywheel.set(speed);
  }

  /**
   * Get the position of the top flywheel
   * @return The position of the top flywheel in ticks
   */
  public double getTopFlywheelPosition() {
    return topFlywheelEncoder.getPosition();
  }

  /**
   * Get the position of the bottom flywheel
   * @return The position of the bottom flywheel in ticks
   */
  public double getBottomFlywheelPosition() {
    return bottomFlywheelOneEncoder.getPosition();
  }
/**
 * Get the position of the kickwheel
 * @return The position of the kickwheel in ticks
 */
  public double getKickwheelPosition() {
    return kickwheelEncoder.getPosition();
  }

  /**
   * Get the velocity of the top flywheel
   * @return The velocity of the top flywheel in meters per second
   */
  public double getTopFlywheelVelocity() {
    return topFlywheelEncoder.getVelocity();
  }

  /**
   * Get the velocity of the bottom flywheel
   * @return The velocity of the bottom flywheel in meters per second
   */
  public double getBottomFlywheelVelocity() {
    return bottomFlywheelOneEncoder.getVelocity();
  }

  /**
   * Get the velocity of the kickwheel
   * @return The velocity of the kickwheel in meters per second
   */
  public double getKickwheelVelocity() {
    return kickwheelEncoder.getVelocity();
  }

  /**
   * Stop the top flywheel
   */
  public void stopTopFlywheel() {
    topFlywheel.set(0);
  }

  /**
   * Stop the bottom flywheel
   */
  public void stopBottomFlywheel() {
    bottomFlywheel.set(0);
  }

  /**
   * Stop the kickwheel
   */
  public void stopKickwheel() {
    kickwheel.set(0);
  }

  /**
   * Stop both flywheels
   */
  public void stopBothFlywheels() {
    stopTopFlywheel();
    stopBottomFlywheel();
  }

  /**
   * Stop both flywheels and the kickwheel
   */
  public void stopAll() {
    stopTopFlywheel();
    stopBottomFlywheel();
    stopKickwheel();
  }

  //TODO add PID methods

  //TODO add "set velocity (RPM)" methods

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
