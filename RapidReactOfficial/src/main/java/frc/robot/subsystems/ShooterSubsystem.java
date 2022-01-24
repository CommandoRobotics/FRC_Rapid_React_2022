// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConstantInversions;
import frc.robot.constants.ConstantPorts;

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
    topFlywheel = new CANSparkMax(ConstantPorts.topFlywheelId, MotorType.kBrushless);
    bottomFlywheelOne = new CANSparkMax(ConstantPorts.bottomFlywheelOneId, MotorType.kBrushless);
    bottomFlywheelTwo = new CANSparkMax(ConstantPorts.bottomFlywheelTwoId, MotorType.kBrushless);
    bottomFlywheelThree = new CANSparkMax(ConstantPorts.bottomFlywheelThreeId, MotorType.kBrushless);
    kickwheel = new CANSparkMax(ConstantPorts.kickwheelId, MotorType.kBrushless);

    // Set motor controller inversions
    topFlywheel.setInverted(ConstantInversions.isTopFlywheelInverted);
    bottomFlywheelOne.setInverted(ConstantInversions.isBottomFlywheelOneInverted);
    bottomFlywheelTwo.setInverted(ConstantInversions.isBottomFlywheelTwoInverted);
    bottomFlywheelThree.setInverted(ConstantInversions.isBottomFlywheelThreeInverted);
    kickwheel.setInverted(ConstantInversions.isKickwheelInverted);

    // Instantiate the encoders
    topFlywheelEncoder = topFlywheel.getEncoder();
    bottomFlywheelOneEncoder = bottomFlywheelOne.getEncoder();
    bottomFlywheelTwoEncoder = bottomFlywheelTwo.getEncoder();
    bottomFlywheelThreeEncoder = bottomFlywheelThree.getEncoder();
    kickwheelEncoder = kickwheel.getEncoder();

    // Set encoder inversions
    topFlywheelEncoder.setInverted(ConstantInversions.isTopFlywheelEncoderInverted);
    bottomFlywheelOneEncoder.setInverted(ConstantInversions.isBottomFlywheelOneEncoderInverted);
    bottomFlywheelTwoEncoder.setInverted(ConstantInversions.isBottomFlywheelTwoEncoderInverted);
    bottomFlywheelThreeEncoder.setInverted(ConstantInversions.isBottomFlywheelThreeEncoderInverted);
    kickwheelEncoder.setInverted(ConstantInversions.isKickwheelEncoderInverted);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
