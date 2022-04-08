// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsField;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;
import frc.robot.Robot;
import frc.robot.Projectile.Range;
import frc.robot.Projectile.Vector;

public class ShooterSubsystem extends SubsystemBase {

  // Declare different Limelight modes
  private final int LED_MODE_DEFAULT = 0;
  private final int LED_MODE_OFF = 1;
  //private final int LED_MODE_BLINK = 2;
  private final int LED_MODE_ON = 3;
  private final int CAM_MODE_VISION_PROCESSOR = 0;
  private final int CAM_MODE_DRIVER_CAMERA = 1;
  private final int SNAPSHOT_MODE_DISABLED = 0;
  private final int SNAPSHOT_MODE_ENABLED = 1;

  // Motor controllers
  CANSparkMax flywheelLeader, flywheelFollower;

  // Encoders
  RelativeEncoder flywheelEncoder;

  // Limelight
  NetworkTable limelight;

  // PID Controllers
  SparkMaxPIDController flywheelPid;

  // Feedforward
  SimpleMotorFeedforward flywheelFF;

  // Some variables that are used locally for inter-method logic
  private int flywheelAtVelocityIteration = 0;
  private double currentTargetRpm = 0;

  double currentManualVelocity = 0;

  private NetworkTable sensorTable, vectorMapNT;

  SlewRateLimiter rateLimiter;


  public ShooterSubsystem() {
    // Instantiate the shooter motor controllers
    flywheelLeader = new CANSparkMax(ConstantsPorts.flywheelLeaderPort, MotorType.kBrushless);
    flywheelFollower = new CANSparkMax(ConstantsPorts.flywheelFollowerPort, MotorType.kBrushless);

    // Reset the sparks
    flywheelLeader.restoreFactoryDefaults();
    flywheelFollower.restoreFactoryDefaults();

    // Set the motor controllers to coast mode
    flywheelLeader.setIdleMode(IdleMode.kCoast);
    flywheelFollower.setIdleMode(IdleMode.kCoast);

    // Set motor controller inversions
    flywheelLeader.setInverted(true);
    // Other flywheel inversions are set in the section below
    flywheelFollower.follow(flywheelLeader, true);

    // Set the spark's current limit
    //flywheelLeader.setSmartCurrentLimit(ConstantsValues.flywheelCurrentLimit);
    //flywheelFollower.setSmartCurrentLimit(ConstantsValues.flywheelCurrentLimit);

    // Instantiate the encoders
    flywheelEncoder = flywheelLeader.getEncoder();
    
    // Set conversion factors
    flywheelEncoder.setVelocityConversionFactor(ConstantsValues.flywheelVelocityConversionFactor);

    // Instantiate the PID controllers
    flywheelPid = flywheelLeader.getPIDController();

    // Set PID controller values
    flywheelPid.setP(ConstantsValues.flywheelP);
    flywheelPid.setI(ConstantsValues.flywheelI);
    flywheelPid.setD(ConstantsValues.flywheelD);
    flywheelPid.setIZone(ConstantsValues.flywheelIZone);
    flywheelPid.setFF(ConstantsValues.flywheelFF);
    flywheelPid.setOutputRange(ConstantsValues.flywheelMinOutput, ConstantsValues.flywheelMaxOutput);

    // Set the ramp rates of the flywheel
    rateLimiter = new SlewRateLimiter(ConstantsValues.flywheelSecondsToSpinUp, 0);
    //flywheelLeader.setClosedLoopRampRate(ConstantsValues.flywheelSecondsToSpinUp);
    //flywheelLeader.setOpenLoopRampRate(ConstantsValues.flywheelSecondsToSpinUp);

    // Instantiate motor feedforwards
    flywheelFF = new SimpleMotorFeedforward(ConstantsValues.flywheelKs, ConstantsValues.flywheelKv);
    
    // Instantiate the limelight
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    sensorTable = NetworkTableInstance.getDefault().getTable("CommandoDash").getSubTable("SensorData");
    vectorMapNT = NetworkTableInstance.getDefault().getTable("CommandoDash").getSubTable("VectorMap");

    // Create a default angle for the vectors below
    double defaultShotAngle = 15;

    // Add ranges and vectors to the vector treemap
    // Note: Velocities are in RPM, angles are in degrees, and ranges are in meters.
    ConstantsValues.addToVectorMap(0, 2, 2050, defaultShotAngle); // Default map for short distances
    ConstantsValues.addToVectorMap(2, 2.5, 2050, defaultShotAngle);
    ConstantsValues.addToVectorMap(2.5, 3, 2200, defaultShotAngle);
    ConstantsValues.addToVectorMap(3, 3.5, 2400, defaultShotAngle);
    ConstantsValues.addToVectorMap(3.5, 4, 2650, defaultShotAngle);
    ConstantsValues.addToVectorMap(4, 4.5, 2775, defaultShotAngle);
    ConstantsValues.addToVectorMap(4.5, 5, 3050, defaultShotAngle);
    ConstantsValues.addToVectorMap(5, 5.5, 3275, defaultShotAngle);
    ConstantsValues.addToVectorMap(5.5, 6, 3700, defaultShotAngle);
    ConstantsValues.addToVectorMap(6, 6.25, 3750, defaultShotAngle);
    ConstantsValues.addToVectorMap(6.25, 6.5, 3950, defaultShotAngle);
    ConstantsValues.addToVectorMap(6.5, 7, 4000, defaultShotAngle);
    ConstantsValues.addToVectorMap(7, 7.5, 4150, defaultShotAngle);
    ConstantsValues.addToVectorMap(7.5, 8, 4150, defaultShotAngle);
    ConstantsValues.addToVectorMap(8, 8.5, 4350, defaultShotAngle);
    ConstantsValues.addToVectorMap(8.5, 200, 4350, defaultShotAngle); // Default map for long distances

    //Add vectormap to CDD
    for(Map.Entry<Range,Vector> entry : ConstantsValues.vectorMap.entrySet()) {
      Range currRange = entry.getKey();
      Vector currVector = entry.getValue();
      vectorMapNT.getEntry(currRange.minValue + " - " + currRange.maxValue).setDouble(currVector.velocity);
    }

    // Add motors to the simulation
    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(flywheelLeader, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(flywheelFollower, DCMotor.getNEO(1));
    }

    // Add values to smart dashboard
    SmartDashboard.putNumber("flywheelP", ConstantsValues.flywheelP);
    SmartDashboard.putNumber("flywheelI", ConstantsValues.flywheelI);
    SmartDashboard.putNumber("flywheelD", ConstantsValues.flywheelD);
    SmartDashboard.putNumber("LLAngle", ConstantsValues.limelightMountingAngle);
    SmartDashboard.putNumber("LLHeight", ConstantsValues.shooterHeightMeters);
    SmartDashboard.putNumber("targetRpm", currentManualVelocity);

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
   * Takes a single snapshot of what the limelight is currently seeing
   */
  public void takeSnapshot() {
    limelight.getEntry("snapshot").setNumber(1);
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
   * Set the RPM of the  flywheel
   * @param target The target RPM of the flywheel.
   */
  public void setFlywheelTargetRpm(double targetRPM) {
    currentTargetRpm = targetRPM;
    if(targetRPM > 0) {
    flywheelPid.setReference(
      rateLimiter.calculate(targetRPM), 
      ControlType.kVelocity, 
      0, 
      flywheelFF.calculate(rateLimiter.calculate(targetRPM)), 
      ArbFFUnits.kVoltage);
    } else {
      flywheelFollower.stopMotor();
    }
    SmartDashboard.putNumber("bottomFlywheelFF", flywheelFF.calculate(rateLimiter.calculate(targetRPM)));
  }

  /**
   * Stop the bottom flywheel
   */
  public void stop() {
    flywheelLeader.set(0);
    currentTargetRpm = 0;
  }
  
  /**
   * Get the position of the flywheel in rotations
   * @return The position of the flywheel in rotations
   */
  public double getFlywheelPosition() {
    return flywheelEncoder.getPosition();
  }

  /**
   * Get the velocity of the flywheel in RPM
   * @return The velocity of the flywheel in RPM
   */
  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity();
  }

  /**
   * Reset the encoder of the flywheel
   */
  public void resetFlywheelEncoder() {
    flywheelEncoder.setPosition(0);
  }

  /**
   * Find the range object in the treemap that contains the specified distance.
   * @param distance The distance, in meters, from the hub.
   * @return The range object from the vector treemap containing that distance. 
   * Returns null if no range containing the given distance exists in the treemap.
   */
  public Range findRangeGivenDistance(double distance) {
    for(Range i : ConstantsValues.vectorMap.keySet()) {
      if(i.isValueInRange(distance)) {
        return i;
      }
    } 
    return null;
  }

  /**
   * Calculate the ideal launch vector.
   * @return The ideal launch vector. This will return a vector with no magnitude
   * or direction if no target is found, or if an ideal vector does not exist
   * for the given distance.
   */
  public Vector calculateIdealLaunchVector() {
    // Capture the distance to the hub before it fluctuates
    double distanceToHubMeters = getHorizontalDistanceToHub();

    // Make sure we can see a target
    if(getHorizontalDistanceToHub() <= 0) {
      // We can not see the target, so return a vector with no magnitude or direction.
      return new Vector(0, 0);
    }

    // Find the shot range our current distance fits into
    Range shotRange = findRangeGivenDistance(distanceToHubMeters);

    // The shot range will be null if no range exists in the treemap
    // for the given distance. In this case, we should return a 
    // vector with no magnitude or direction.
    if(shotRange == null) {
      return new Vector(0, 0);
    }

    // Find the vector for that range
    Vector idealVector = ConstantsValues.vectorMap.get(shotRange);

    return idealVector;
  }

  /**
   * Cycle the current manual velocity
   */
  public void cycleManualVelocity() {
    System.out.println("Getting to cycle manual velocity");
    if(currentManualVelocity == 0) {
      currentManualVelocity = 1000;
    } else if(currentManualVelocity == 1000) {
      currentManualVelocity = 1500;
    } else if(currentManualVelocity == 1500) {
      currentManualVelocity = 2000;
    } else if(currentManualVelocity == 2000) {
      currentManualVelocity = 2500;
    } else if(currentManualVelocity == 2500) {
      currentManualVelocity = 3000;
    } else if(currentManualVelocity == 3000) {
      currentManualVelocity = 4000;
    } else {
      currentManualVelocity = 0;
    }
    SmartDashboard.putNumber("targetRpm", currentManualVelocity);
  }

  /**
   * Get the current manual velocity
   * @return
   */
  public double getCurrentManualVelocity() {
    return currentManualVelocity;
  }

  public boolean isFlywheelAtTargetVelocity() {
    return flywheelAtVelocityIteration >=ConstantsValues.flywheelAtVelocityIterations && currentTargetRpm != 0;
  }

  /**
   * Update the network tables that integrate our shooter with CommandoDash
   */
  private void updateCommandoDash() {
    sensorTable.getEntry("manualCycleSpeed").setDouble(getCurrentManualVelocity());
    sensorTable.getEntry("targetRPM").setDouble(currentTargetRpm);
    sensorTable.getEntry("shooterRPM").setDouble(getFlywheelVelocity());
    sensorTable.getEntry("isAtTargetVelocity").setBoolean(isFlywheelAtTargetVelocity());
    for(Map.Entry<Range,Vector> entry : ConstantsValues.vectorMap.entrySet()) {
      Range currRange = entry.getKey();
      Vector currVector = entry.getValue();
      double newRPM = vectorMapNT.getEntry(currRange.minValue + " - " + currRange.maxValue).getDouble(currVector.velocity);
      if (newRPM != currVector.velocity) {
        currVector.velocity = newRPM;
        System.out.println("Updated VectorMap in Range " + currRange.minValue + " - " + currRange.maxValue + " to " + newRPM + " successfully");
      }
    }
  }

  @Override
  public void periodic() {

    // Enable vision processing mode
    enableLimelightVisionProcessing();

    // Update whether the flywheel is at the target velocity
    if((getFlywheelVelocity() > currentTargetRpm-ConstantsValues.flywheelAtVelocityDeadband) && (getFlywheelVelocity() < currentTargetRpm+ConstantsValues.flywheelAtVelocityDeadband)) {
      flywheelAtVelocityIteration++;
    } else {
      flywheelAtVelocityIteration = 0;
    }

    // Update CommandoDash
    updateCommandoDash();

    // Write to smart dashboard
    currentManualVelocity = SmartDashboard.getNumber("manualTargetVel", currentManualVelocity);
    SmartDashboard.putNumber("horizontalDistanceLL", getHorizontalDistanceToHub());
    ConstantsValues.limelightMountingAngle = SmartDashboard.getNumber("LLAngle", ConstantsValues.limelightMountingAngle);
    ConstantsValues.shooterHeightMeters = SmartDashboard.getNumber("LLHeight", ConstantsValues.shooterHeightMeters);
    flywheelPid.setP(SmartDashboard.getNumber("flywheelP", ConstantsValues.flywheelP));
    ConstantsValues.flywheelI = SmartDashboard.getNumber("flywheelI", ConstantsValues.flywheelI);
    flywheelPid.setD(SmartDashboard.getNumber("flywheelD", ConstantsValues.flywheelD));
    SmartDashboard.putNumber("ActualRPM", getFlywheelVelocity());
    
  }

  @Override
  public void simulationPeriodic() {
    // Update Spark Maxes
    REVPhysicsSim.getInstance().run();
  }

}
