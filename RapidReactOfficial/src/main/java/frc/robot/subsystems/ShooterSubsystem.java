// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private final int LED_MODE_BLINK = 2;
  private final int LED_MODE_ON = 3;
  private final int CAM_MODE_VISION_PROCESSOR = 0;
  private final int CAM_MODE_DRIVER_CAMERA = 1;
  private final int SNAPSHOT_MODE_DISABLED = 0;
  private final int SNAPSHOT_MODE_ENABLED = 1;

  // Motor controllers
  CANSparkMax bottomLeader, bottomFollowerOne, bottomFollowerTwo, top, kickwheel;

  // Encoders
  RelativeEncoder bottomEncoder, topEncoder, kickwheelEncoder;

  // Limelight
  NetworkTable limelight;

  // PID Controllers
  SparkMaxPIDController bottomPid, topPid, kickwheelPid;

  // Feedforward
  SimpleMotorFeedforward topFF, bottomFF, kickwheelFF;

  double previousBottomP = ConstantsValues.bottomFlywheelP;
  double previousTopP = ConstantsValues.topFlywheelP;
  double previousKickwheelP = ConstantsValues.kickwheelP;

  double currentManualVelocity = 0;

  public ShooterSubsystem() {
    // Instantiate the shooter motor controllers
    bottomLeader = new CANSparkMax(ConstantsPorts.bottomLeaderPort, MotorType.kBrushless);
    bottomFollowerOne = new CANSparkMax(ConstantsPorts.bottomFollowerOnePort, MotorType.kBrushless);
    bottomFollowerTwo = new CANSparkMax(ConstantsPorts.bottomFollowerTwoPort, MotorType.kBrushless);
    top = new CANSparkMax(ConstantsPorts.topPort, MotorType.kBrushless);
    kickwheel = new CANSparkMax(ConstantsPorts.kickwheelPort, MotorType.kBrushless);


    // Reset the sparks
    bottomLeader.restoreFactoryDefaults();
    bottomFollowerOne.restoreFactoryDefaults();
    bottomFollowerTwo.restoreFactoryDefaults();
    top.restoreFactoryDefaults();
    kickwheel.restoreFactoryDefaults();

    // Set the motor controllers to coast mode
    bottomLeader.setIdleMode(IdleMode.kCoast);
    bottomFollowerOne.setIdleMode(IdleMode.kCoast);
    bottomFollowerTwo.setIdleMode(IdleMode.kCoast);
    top.setIdleMode(IdleMode.kCoast);
    kickwheel.setIdleMode(IdleMode.kCoast);

    // Set motor controller inversions
    //TODO set the proper motor inversions
    bottomLeader.setInverted(false);
    // Other bottom flywheel inversions are set in the section below
    top.setInverted(false);
    kickwheel.setInverted(false);

    // Set all bottom flywheel motor controllers to follow the main.
    // The boolean value represents whether the motor controller that is following
    // should be inverted relative to the main motor controller.
    bottomFollowerOne.follow(bottomLeader, false);
    bottomFollowerTwo.follow(bottomLeader, false);

    // Instantiate the encoders
    bottomEncoder = bottomLeader.getEncoder();
    topEncoder = top.getEncoder();
    kickwheelEncoder = kickwheel.getEncoder();

    // Instantiate the PID controllers
    bottomPid = bottomLeader.getPIDController();
    topPid = top.getPIDController();
    kickwheelPid = kickwheel.getPIDController();

    // Set PID controller values
    bottomPid.setP(ConstantsValues.bottomFlywheelP);
    bottomPid.setI(ConstantsValues.bottomFlywheelI);
    bottomPid.setD(ConstantsValues.bottomFlywheelD);
    bottomPid.setIZone(ConstantsValues.bottomFlywheelIZone);
    bottomPid.setFF(ConstantsValues.bottomFlywheelFF);
    bottomPid.setOutputRange(ConstantsValues.bottomFlywheelMinOutput, ConstantsValues.bottomFlywheelMaxOutput);
    topPid.setP(ConstantsValues.topFlywheelP);
    topPid.setI(ConstantsValues.topFlywheelI);
    topPid.setD(ConstantsValues.topFlywheelD);
    topPid.setIZone(ConstantsValues.topFlywheelIZone);
    topPid.setFF(ConstantsValues.topFlywheelFF);
    topPid.setOutputRange(ConstantsValues.topFlywheelMinOutput, ConstantsValues.topFlywheelMaxOutput);
    kickwheelPid.setP(ConstantsValues.kickwheelP);
    kickwheelPid.setI(ConstantsValues.kickwheelI);
    kickwheelPid.setD(ConstantsValues.kickwheelD);
    kickwheelPid.setIZone(ConstantsValues.kickwheelIZone);
    kickwheelPid.setFF(ConstantsValues.kickwheelFF);
    kickwheelPid.setOutputRange(ConstantsValues.kickwheelMinOutput, ConstantsValues.kickwheelMaxOutput);

    // Instantiate motor feedforwards
    bottomFF = new SimpleMotorFeedforward(ConstantsValues.bottomFlywheelKs, ConstantsValues.bottomFlywheelKv, ConstantsValues.bottomFlywheelKa);
    topFF = new SimpleMotorFeedforward(ConstantsValues.topFlywheelKs, ConstantsValues.topFlywheelKv, ConstantsValues.topFlywheelKa);
    kickwheelFF = new SimpleMotorFeedforward(ConstantsValues.kickwheelKs, ConstantsValues.kickwheelKv, ConstantsValues.kickwheelKa);

    // Instantiate the limelight
    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // Create a default angle for the vectors below
    double defaultShotAngle = 15;

    // Add ranges and vectors to the vector treemap
    // Note: Velocities are in RPM, angles are in degrees, and ranges are in meters.
    ConstantsValues.addToVectorMap(0, 3, 2000, defaultShotAngle);
    ConstantsValues.addToVectorMap(3, 10, 4000, defaultShotAngle);
    //TODO adjust the above vectors, as they're just examples

    // Add motors to the simulation
    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(bottomLeader, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(bottomFollowerOne, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(kickwheel, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(top, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(kickwheel, DCMotor.getNEO(1));
    }
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
  public void setBottomVoltage(double volts) {
    bottomLeader.setVoltage(volts);
  }

  /**
   * Set the voltage of the top flywheel
   * @param volts
   */
  public void setTopVoltage(double volts) {
    top.setVoltage(volts);
  }

  /**
   * Set the voltage of the kickwheel
   * @param volts
   */
  public void setKickwheelVoltage(double volts) {
    kickwheel.setVoltage(volts);
  }

  /**
   * Set the RPM of the bottom flywheel
   * @param target The target RPM of the flywheel.
   */
  public void setBottomTargetRpm(double targetRPM) {
    bottomPid.setReference(
      targetRPM, 
      ControlType.kVelocity, 
      0, 
      bottomFF.calculate(targetRPM), 
      ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("bottomFlywheelFF", bottomFF.calculate(targetRPM));
  }

  /**
   * Set the RPM of the top flywheel
   * @param target The target RPM of the top flywheel
   */
  public void setTopTargetRPM(double targetRPM) {
    topPid.setReference(
      targetRPM, 
      ControlType.kVelocity, 
      0, 
      topFF.calculate(targetRPM), 
      ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("topFlywheelFF", topFF.calculate(targetRPM));
  }

  /**
   * Set the RPM of the kickwheel
   * @param target The target RPM of the kickwheel
   */
  public void setKickwheelTargetRpm(double targetRPM) {
    kickwheelPid.setReference(
      targetRPM, 
      ControlType.kVelocity, 
      0, 
      kickwheelFF.calculate(targetRPM), 
      ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("kickwheelFF", kickwheelFF.calculate(targetRPM));
  }

  /**
   * Stop the bottom flywheel
   */
  public void stopBottom() {
    bottomLeader.set(0);
  }

  /**
   * Stop the top flywheel
   */
  public void stopTop() {
    top.set(0);
  }

  /**
   * Stop the kickwheel
   */
  public void stopKickwheel() {
    kickwheel.set(0);
  }

  /**
   * Stop both the top and bottom flywheels
   */
  public void stopBothFlywheels() {
    bottomLeader.set(0);
    top.set(0);
  }

  /**
   * Stop all wheels, including the bottom flywheel, the top flywheel, and the kickwheel
   */
  public void stopAll() {
    bottomLeader.set(0);
    top.set(0);
    kickwheel.set(0);
  }
  
  //TODO add correct units to the below method descriptions
  /**
   * Get the position of the bottom flywheel
   * @return The position of the bottom flywheel
   */
  public double getBottomPosition() {
    return bottomEncoder.getPosition();
  }

  /**
   * Get the position of the top flywheel
   * @return The position of the top flywheel
   */
  public double getTopPosition() {
    return topEncoder.getPosition();
  }

  /**
   * Get the position of the kickwheel
   * @return The position of the kickwheen
   */
  public double getKickwheelPosition() {
    return kickwheelEncoder.getPosition();
  }

  /**
   * Get the velocity of the bottom flywheel in RPM
   * @return The velocity of the bottom flywheel in RPM
   */
  public double getBottomVelocity() {
    return bottomEncoder.getVelocity();
  }

  /**
   * Get the velocity of the top flywheel in RPM
   * @return The velocity of the top flywheel in RPM
   */
  public double getTopVelocity() {
    return topEncoder.getVelocity();
  }

  /**
   * Get the velocity of the kickwheel in RPM
   * @return
   */
  public double getKickwheelVelocity() {
    return kickwheelEncoder.getVelocity();
  }

  /**
   * Reset the encoder of the bottom flywheel
   */
  public void resetBottomEncoder() {
    bottomEncoder.setPosition(0);
  }

  /**
   * Reset the encoder of the top flywheel
   */
  public void resetTopEncoder() {
    topEncoder.setPosition(0);
  }

  /**
   * Reset the encoder of the kickwheel
   */
  public void resetKickwheelEncoder() {
    kickwheelEncoder.setPosition(0);
  }

  /**
   * Reset the encoders of the bottom and top flywheels
   */
  public void resetFlywheelEncoders() {
    bottomEncoder.setPosition(0);
    topEncoder.setPosition(0);
  }

  /**
   * Reset all shooter encoders
   */
  public void resetAllEncoders() {
    bottomEncoder.setPosition(0);
    topEncoder.setPosition(0);
    kickwheelEncoder.setPosition(0);
  }

  // Update P values for top and bottom flywheel in periodc

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
    if(getHorizontalDistanceToHub() == -1) {
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
    if(currentManualVelocity == 0) {
      currentManualVelocity = 1000;
    } else if(currentManualVelocity == 1000) {
      currentManualVelocity = 2000;
    } else if(currentManualVelocity == 2000) {
      currentManualVelocity = 3000;
    } else if(currentManualVelocity == 3000) {
      currentManualVelocity = 4000;
    } else if(currentManualVelocity == 4000) {
      currentManualVelocity = 5000;
    } else if(currentManualVelocity == 5000) {
      currentManualVelocity = 6000;
    } else {
      currentManualVelocity = 0;
    }
  }

  /**
   * Get the current manual velocity
   * @return
   */
  public double getCurrentManualVelocity() {
    return currentManualVelocity;
  }

  @Override
  public void periodic() {
    // Get the current flywheel values from the Smart dashboard
    // This will allow us to adjust these values on the fly from the Smart Dashboard
    double currentTopP = SmartDashboard.getNumber("topP", 0);
    double currentBottomP = SmartDashboard.getNumber("bottomP", 0);
    double currentKickwheelP = SmartDashboard.getNumber("kickwheelP", 0);

    // Check if the numbers on the Smart Dashboard have changed, and change our values if they have.
    if (currentTopP != previousTopP) {
      topPid.setP(currentTopP);
      previousTopP = currentTopP;
    }

    if(currentBottomP != previousBottomP) {
      bottomPid.setP(currentBottomP);
      previousBottomP = currentBottomP;
    }

    if(currentKickwheelP != previousKickwheelP) {
      kickwheelPid.setP(currentKickwheelP);
      previousKickwheelP = currentKickwheelP;
    }
  }

  @Override
  public void simulationPeriodic() {
    // Update Spark Maxes
    REVPhysicsSim.getInstance().run();
  }

}
