// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Climbing;

public class ClimberSubsystem extends SubsystemBase {

  CANSparkMax leftWinch;
  CANSparkMax rightWinch;
  CANSparkMax tiltMotor;

  SparkMaxPIDController winchControl;
  SparkMaxPIDController tiltControl;
  
  DigitalInput fullyRetractedLimitSwitch;
  DigitalInput fullyTiltedDownLimitSwitch;
  // TODO: Uncomment this when a port is assigned for this limit switch.
  //DigitalInput fullyTiltedUpLimitSwitch;

  ClimbSequence autoClimb;

  // TODO: Update NetworkTables for current information.
  NetworkTableInstance ntInst;
  boolean previousMidState = false;
  boolean previousTraversalState = false;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Initiate SparkMaxes
    rightWinch = new CANSparkMax(ConstantsPorts.rightWinchId, MotorType.kBrushless);
    leftWinch = new CANSparkMax(ConstantsPorts.leftWinchId, MotorType.kBrushless);
    tiltMotor = new CANSparkMax(ConstantsPorts.armTiltId, MotorType.kBrushless);

    // Set Inversions and following
    rightWinch.setInverted(false); // Extending (releasing cord) is positive.
    // Set left Winch to follow right-side (shooter is front).
    leftWinch.follow(rightWinch, true);
    tiltMotor.setInverted(true); // Tilting up/toward front (shooter) is positive.

    // Set PID
    SparkMaxPIDController winchControl = rightWinch.getPIDController();
    winchControl.setP(1);
    winchControl.setI(0);
    winchControl.setD(0);
    winchControl.setIZone(0);
    winchControl.setFF(0);
    winchControl.setOutputRange(-1, 1);

    SparkMaxPIDController tiltControl = tiltMotor.getPIDController();
    tiltControl.setP(1);
    tiltControl.setI(0);
    tiltControl.setD(0);
    tiltControl.setIZone(0);
    tiltControl.setFF(0);
    tiltControl.setOutputRange(-1, 1);

    DigitalInput fullyRetractedLimitSwitch = new DigitalInput(ConstantsPorts.fullyRetractedLimitSwitchPort);
    DigitalInput fullyTiltedDownLimitSwitch = new DigitalInput(ConstantsPorts.titledDownLimitSwitchPort);
    // TODO: Uncomment this when a port is assigned for this limit.
    //DigitalInput fullyTiltedUpLimitSwitch = new DigitalInput(ConstantsPorts.titledUpLimitSwitchPort);  

    autoClimb = new ClimbSequence();

    // TODO: Add entries in NetworkTables for Climber
    //ntInst = NetworkTableInstance.getDefault();
    //ntInst.getTable("CommandoDash").getSubTable("SensorData")
    //  .getEntry("midSolenoidState").setBoolean(isMidClimbExtended());

  }
  
  // Call this (once) to advance the climb sequence (when you are happy with where the arms are) so the arms move to the next position.
  public void moveToNextClimbPosition() {
    autoClimb.advance();
    setClimberMotorsToTargetPosition();
  }

  // Call this (once) to go back to the previous the climb position.
  public void backToPreviousClimbPosition() {
    autoClimb.goBack();
    setClimberMotorsToTargetPosition();
  }

  public void stopMovement() {
    winchControl.setReference(0, CANSparkMax.ControlType.kVelocity);
    tiltControl.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  // Call repeatedly (i.e. hold button) to let the robot perform the entire climb sequence
  public void justClimb() {
    double winchPositionInEncoderTicks = rightWinch.getEncoder().getPosition();
    double tiltPositionInEncoderTicks = tiltMotor.getEncoder().getPosition();
    ClimberPosition currentPosition = ClimberPosition.fromEncoderValues(winchPositionInEncoderTicks, tiltPositionInEncoderTicks);
    if (autoClimb.closeEnough(currentPosition)) {
      autoClimb.advance();
      setClimberMotorsToTargetPosition();
    }
  }

  public void extendArm() {
    if (!ClimbLimits.safeToExtend(getCurrentPosition())) {
      winchControl.setReference(0, CANSparkMax.ControlType.kVelocity);
    } else {
      // If calling this probably making minor adjustments.
      // TODO: Tune this value.
      final double outputShaftSpeedInRPM = 60;
      final double motorSpeedInRPM = outputShaftSpeedInRPM * ClimberSpecs.winchGearboxRatio;
      winchControl.setReference(motorSpeedInRPM, CANSparkMax.ControlType.kVelocity);
    }
  }

  public void retractArm() {
    if (fullyRetractedLimitSwitch.get()) {
      winchControl.setReference(0, CANSparkMax.ControlType.kVelocity);
    } else {
      // If calling this probably making minor adjustments.
      // TODO: Tune this value.
      final double outputShaftSpeedInRPM = -60;
      final double motorSpeedInRPM = outputShaftSpeedInRPM * ClimberSpecs.winchGearboxRatio;
      winchControl.setReference(motorSpeedInRPM, CANSparkMax.ControlType.kVelocity);
    }
  }

  public void stopArmWinch() {
    winchControl.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public void tiltArmUp() {
    if (!safeToTiltUp(getCurrentPosition()) {
      tiltControl.setReference(0, CANSparkMax.ControlType.kVelocity);
    } else {
      // If calling this probably making minor adjustments.
      // TODO: Tune this value.
      final double outputShaftSpeedInRPM = 2;
      final double motorSpeedInRPM = outputShaftSpeedInRPM * ClimberSpecs.tiltGearboxRatio;
      tiltControl.setReference(motorSpeedInRPM, CANSparkMax.ControlType.kVelocity);
    }
  }

  public void tiltArmDown() {
    if (!safeToTiltDown(getCurrentPosition()) {
      tiltControl.setReference(0, CANSparkMax.ControlType.kVelocity);
    } else {
      // If calling this probably making minor adjustments.
      // TODO: Tune this value.
      final double outputShaftSpeedInRPM = -2;
      final double motorSpeedInRPM = outputShaftSpeedInRPM * ClimberSpecs.tiltGearboxRatio;
      tiltControl.setReference(motorSpeedInRPM, CANSparkMax.ControlType.kVelocity);
    }
  }

  public void stopArmTilt() {
    tiltControl.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * Manually set the power of the winch motors (i.e. from joystick).
   * @param power The desired power from -1 (retract at full power) to +1 (extend at full power)
   * @param applyLimits If true, extension/retraction will be stopped if it would move beyond legal bounds, or limit switches are hit. False allows you to override these.
   */
  public void setWinchPower(double power, boolean applyLimits) {
    if (power > 1.0) {
      power = 1.0;
    }
    if (power < -1.0) {
      power = -1.0;
    }
    // Apply logic to prevent the winch from doing bad things.
    if (applyLimits) {
      if (power > 0) {
        // Are we allowed to extend?
        if (!ClimbLimits.canExtend(getCurrentPosition())) {
          // Don't let it go further.
          power = 0;
        }
      } else {
        // Are we allowed to retract?
        if (fullyRetractedLimitSwitch.get()) {
          // Already hit the switch, don't go further.
          power = 0;
        }
      }
    }
    rightWinch.set(power);
  }

  /**
   * Manually set the power of the tilt motor (i.e. from joystick).
   * @param power The desired power from -1 (down at full power) to +1 (up at full power)
   * @param applyLimits If true, motion will be stopped if it would move beyond legal bounds, or limit switches are hit. False allows you to override these.
   */
  public void setTiltPower(double power, boolean applyLimits) {
    if (power > 1.0) {
      power = 1.0;
    }
    if (power < -1.0) {
      power = -1.0;
    }
    // Apply logic to prevent the winch from doing bad things.
    if (applyLimits) {
      if (power > 0) {
        // Are we allowed to tilt up?
        if (!ClimbLimits.canTiltUp(getCurrentPosition())) {
          // Don't let it go further.
          power = 0;
        }
        // TODO: Uncomment this when we have a port assigned for this limit switch
        //if (fullyTiltedUpLimitSwitch.get()) {
        //  power = 0;
        //}
      } else {
        // Are we allowed to tilt down?
        if (!ClimbLimits.canTiltDown(getCurrentPosition())) {
          // Don't let it go further.
          power = 0;
        }
        if (fullyTiltedDownLimitSwitch.get()) {
          power = 0;
        }
      }
    }
    rightWinch.set(power);
  }

  private void setClimberMotorsToTargetPosition() {
    ClimberPosition desiredPosition = autoClimb.getDesiredPosition();
    winchControl.setReference(desiredPosition.winchEncoderTicks(), CANSparkMax.ControlType.kSmartMotion);
    tiltControl.setReference(desiredPosition.tiltEncoderTicks(), CANSparkMax.ControlType.kSmartMotion);
  }

  private ClimberPosition getCurrentPosition() {
    double winchTicks = rightWinch.getEncoder().getPosition();
    double tiltTicks = tiltMotor.getEncoder().getPosition();
    return ClimberPosition.fromEncoderValues(winchTicks, tiltTicks);
  }

  @Override
  public void periodic() {
    // Send updates to ComandoDash
    boolean currMidState = false;
    if (previousMidState != currMidState) {
        ntInst.getTable("CommandoDash").getSubTable("SensorData")
            .getEntry("midSolenoidState").setBoolean(currMidState);
    }
    previousMidState = currMidState;
    // TODO: Add relevant statistics to CommandoDash
  }
}
