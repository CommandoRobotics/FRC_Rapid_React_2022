// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;

public class AutoAimSubsystem extends SubsystemBase {

  // Declare different Limelight modes
  private final int LED_MODE_DEFAULT = 0;
  private final int LED_MODE_OFF = 1;
  //private final int LED_MODE_BLINK = 2;
  private final int LED_MODE_ON = 3;
  private final int CAM_MODE_VISION_PROCESSOR = 0;
  private final int CAM_MODE_DRIVER_CAMERA = 1;
  private final int SNAPSHOT_MODE_DISABLED = 0;
  private final int SNAPSHOT_MODE_ENABLED = 1;

  CANSparkMax pan, tilt;
  RelativeEncoder panEncoder, tiltEncoder;
  PIDController panPid, tiltPid;
  DigitalInput panLimit, tiltLimit;
  NetworkTable limelight;

  public AutoAimSubsystem() {
    // Instantiate sparks
    pan = new CANSparkMax(ConstantsPorts.panId, MotorType.kBrushless);
    tilt = new CANSparkMax(ConstantsPorts.tiltId, MotorType.kBrushless);

    // Instantiate encoders
    panEncoder = pan.getEncoder();
    tiltEncoder = tilt.getEncoder();

    // Set conversion factors
    panEncoder.setPositionConversionFactor(ConstantsValues.panPositionConversionFactor);
    tiltEncoder.setPositionConversionFactor(ConstantsValues.tiltPositionConversionFactor);

    // Instantiate PID
    panPid = new PIDController(ConstantsValues.panP, ConstantsValues.panI, ConstantsValues.panD);
    tiltPid = new PIDController(ConstantsValues.tiltP, ConstantsValues.tiltI, ConstantsValues.tiltD);

    panPid.setSetpoint(ConstantsValues.panSetPoint);
    tiltPid.setSetpoint(ConstantsValues.tiltSetPoint);

    // Instantiate limit switches
    //panLimit = new DigitalInput(ConstantsPorts.panLimitSwitchPort);
    //tiltLimit = new DigitalInput(ConstantsPorts.tiltLimitSwitchPort);

    // Instantiate the limelight
    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    if(Robot.isSimulation()) {
      // Add motors to the RevPhysicsSim if this is a simlated robot
      REVPhysicsSim.getInstance().addSparkMax(pan, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(tilt, DCMotor.getNeo550(1));
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
   * @return The x offset of the target as seen by the limelight in pixels.
   */
  public double getLimelightXOffset() {
    return limelight.getEntry("tx").getDouble(0);
  }

  /**
   * Get the y offset of the target seen by the Limelight.
   * @return The y offset of the target as seen by the limelight in pixels.
   */
  public double getLimelightYOffset() {
    return limelight.getEntry("ty").getDouble(0);
  }

  /**
   * Set the speed of the pan motor
   * @param speed
   */
  public void setPan(double speed) {
    pan.set(speed);
  }

  /**
   * Set the speed of the tilt motor
   * @param speed
   */
  public void setTilt(double speed) {
    tilt.set(speed);
  }

  /**
   * Set the voltage of the pan motor
   * @param volts
   */
  public void setPanVoltage(double volts) {
    pan.setVoltage(volts);
  }

  /**
   * Set the voltage of the tilt motor
   * @param volts
   */
  public void setTiltVoltage(double volts) {
    tilt.setVoltage(volts);
  }

  /**
   * Stop the pan motor
   */
  public void stopPan() {
    pan.set(0);
  }

  /**
   * Stop the tilt motor
   */
  public void stopTilt() {
    tilt.set(0);
  }

  /**
   * Stop the pan and tilt motors
   */
  public void stopAll() {
    pan.set(0);
    tilt.set(0);
  }

  /**
   * Get the position of the pan
   * @return
   */
  public double getPanPosition() {
    return panEncoder.getPosition();
  }

  /**
   * Get the position of the tilt
   * @return
   */
  public double getTiltPosition() {
    return tiltEncoder.getPosition();
  }

  /**
   * Get the velocity of the pan
   * @return
   */
  public double getPanVelocity() {
    return panEncoder.getVelocity();
  }

  /**
   * Get the velocity of the tilt
   * @return
   */
  public double getTilteVelocity() {
    return tiltEncoder.getVelocity();
  }

  /**
   * Reset the pan encoder
   */
  public void resetPanEncoder() {
    panEncoder.setPosition(0);
  }

  /**
   * Reset the tilt encoder
   */
  public void resetTiltEncoder() {
    tiltEncoder.setPosition(0);
  }

  /**
   * Get whether the pan limit switch is pressed
   * @return
   */
  public boolean isPanPressed() {
    return panLimit.get();
  }

  /**
   * Get whether the tilt limit switch is pressed
   * @return
   */
  public boolean isTiltPressed() {
    return tiltLimit.get();
  }

  /**
   * Calculate the power to set the motors
   * @param limelightXOffset
   * @return
   */
  public double calculatePanOutput(double limelightXOffset) {
    double pidOutput = panPid.calculate(limelightXOffset+ConstantsValues.limelightPanOffset);
    return MathUtil.clamp(
      MathUtil.applyDeadband(pidOutput, ConstantsValues.panPidDeadzone), 
      -ConstantsValues.panPidMaxOutput, 
      ConstantsValues.panPidMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
