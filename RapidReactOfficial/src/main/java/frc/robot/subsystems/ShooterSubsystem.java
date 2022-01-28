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

import edu.wpi.first.math.util.Units;
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
  CANSparkMax topFlywheel, bottomFlywheel, bottomFlywheelTwo, bottomFlywheelThree, kickwheel;

  // Encoders
  RelativeEncoder topFlywheelEncoder, bottomFlywheelEncoder, kickwheelEncoder;

  // Limelight
  NetworkTable limelight;

  SparkMaxPIDController topFlywheelPidController, bottomFlywheelPidController, kickwheelPidController;

  public ShooterSubsystem() {
    // Instantiate the shooter motor controllers
    topFlywheel = new CANSparkMax(ConstantsPorts.topFlywheelId, MotorType.kBrushless);
    bottomFlywheel = new CANSparkMax(ConstantsPorts.bottomFlywheelOneId, MotorType.kBrushless);
    bottomFlywheelTwo = new CANSparkMax(ConstantsPorts.bottomFlywheelTwoId, MotorType.kBrushless);
    bottomFlywheelThree = new CANSparkMax(ConstantsPorts.bottomFlywheelThreeId, MotorType.kBrushless);
    kickwheel = new CANSparkMax(ConstantsPorts.kickwheelId, MotorType.kBrushless);

    // Set the motor controllers to coast mode
    topFlywheel.setIdleMode(IdleMode.kCoast);
    bottomFlywheel.setIdleMode(IdleMode.kCoast);
    bottomFlywheelTwo.setIdleMode(IdleMode.kCoast);
    bottomFlywheelThree.setIdleMode(IdleMode.kCoast);
    kickwheel.setIdleMode(IdleMode.kCoast);

    // Set motor controller inversions
    //TODO set the proper motor inversions
    topFlywheel.setInverted(false);
    bottomFlywheel.setInverted(false);
    // Other bottom flywheel inversions are set in the section below
    kickwheel.setInverted(false);

    // Set all bottom flywheel motor controllers to follow the main.
    // The boolean value represents whether the motor controller that is following
    // should be inverted relative to the main motor controller.
    bottomFlywheelTwo.follow(bottomFlywheel, false);
    bottomFlywheelThree.follow(bottomFlywheel, false);

    // Instantiate the encoders
    topFlywheelEncoder = topFlywheel.getEncoder();
    bottomFlywheelEncoder = bottomFlywheel.getEncoder();
    kickwheelEncoder = kickwheel.getEncoder();

    // Set the velocity conversion factors
    topFlywheelEncoder.setVelocityConversionFactor(ConstantsValues.topFlywheelVelocityConversionFactor);
    bottomFlywheelEncoder.setVelocityConversionFactor(ConstantsValues.bottomFlywheelVelocityConversionFactor);
    kickwheelEncoder.setVelocityConversionFactor(ConstantsValues.kickwheelVelocityConversionFactor);

    // Instantiate the PID controllers
    topFlywheelPidController = topFlywheel.getPIDController();
    bottomFlywheelPidController = bottomFlywheel.getPIDController();
    kickwheelPidController = kickwheel.getPIDController();

    // Set PID controller values
    topFlywheelPidController.setP(ConstantsValues.topFlywheelP);
    topFlywheelPidController.setI(ConstantsValues.topFlywheelI);
    topFlywheelPidController.setD(ConstantsValues.topFlywheelD);
    topFlywheelPidController.setIZone(ConstantsValues.topFlywheelIZone);
    topFlywheelPidController.setFF(ConstantsValues.topFlywheelFeedForward);
    topFlywheelPidController.setOutputRange(ConstantsValues.topFlywheelMinOutput, ConstantsValues.topFlywheelMaxOutput);
    bottomFlywheelPidController.setP(ConstantsValues.bottomFlywheelP);
    bottomFlywheelPidController.setI(ConstantsValues.bottomFlywheelI);
    bottomFlywheelPidController.setD(ConstantsValues.bottomFlywheelD);
    bottomFlywheelPidController.setIZone(ConstantsValues.bottomFlywheelIZone);
    bottomFlywheelPidController.setFF(ConstantsValues.bottomFlywheelFeedForward);
    bottomFlywheelPidController.setOutputRange(ConstantsValues.bottomFlywheelMinOutput, ConstantsValues.bottomFlywheelMaxOutput);
    kickwheelPidController.setP(ConstantsValues.kickwheelP);
    kickwheelPidController.setI(ConstantsValues.kickwheelI);
    kickwheelPidController.setD(ConstantsValues.kickwheelD);
    kickwheelPidController.setIZone(ConstantsValues.kickwheelIZone);
    kickwheelPidController.setFF(ConstantsValues.kickwheelFeedForward);
    kickwheelPidController.setOutputRange(ConstantsValues.kickwheelMinOutput, ConstantsValues.kickwheelMaxOutput);

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
    return bottomFlywheelEncoder.getPosition();
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
   * @return The velocity of the top flywheel in meters per second/
   */
  public double getTopFlywheelVelocity() {
    return topFlywheelEncoder.getVelocity();
  }

  /**
   * Get the velocity of the bottom flywheel
   * @return The velocity of the bottom flywheel in meters per second.
   */
  public double getBottomFlywheelVelocity() {
    return bottomFlywheelEncoder.getVelocity();
  }

  /**
   * Get the velocity of the kickwheel
   * @return The velocity of the kickwheel in meters per second.
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

  /**
   * Set the velocity of the bottom flywheel.
   * @param target The target velocity in meters per second.
   */
  public void setBottomFlywheelVelocity(double target) {
    bottomFlywheelPidController.setReference(target, ControlType.kVelocity);
  }

  /**
   * Set the velocity of the top flywheel.
   * @param target The target velocity in meters per second.
   */
  public void setTopFlywheelRPM(double target) {
    topFlywheelPidController.setReference(target, ControlType.kVelocity);
  }

  /**
   * Set the velocity of the kickwheel.
   * @param target The target velocity in meters per second.
   */
  public void setKickwheelRPM(double target) {
    kickwheelPidController.setReference(target, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    
  }
}
