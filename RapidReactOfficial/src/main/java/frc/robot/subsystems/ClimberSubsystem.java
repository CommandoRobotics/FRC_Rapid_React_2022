// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;

public class ClimberSubsystem extends SubsystemBase {

  // Motor controllers
  CANSparkMax winchLeader, winchFollower, tilt;

  // Encoders
  RelativeEncoder winchEncoder, tiltEncoder;

  final double winchMaxSpeed, winchMaxVolts, tiltMaxSpeed, tiltMaxVolts;
  

  public ClimberSubsystem() {

    // Instantiate motor controllers
    winchLeader = new CANSparkMax(ConstantsPorts.winchLeaderId, MotorType.kBrushless);
    winchFollower = new CANSparkMax(ConstantsPorts.winchFollowerId, MotorType.kBrushless);
    tilt = new CANSparkMax(ConstantsPorts.climberTiltId, MotorType.kBrushless);

    // Set motor inversions
    winchLeader.setInverted(false);
    tilt.setInverted(false);
    // Note: Winch follower inversion is done when setting it as a follower below

    // Set the winch follower
    winchFollower.follow(winchLeader, true);

    // Instantiate encoders
    winchEncoder = winchLeader.getEncoder();
    tiltEncoder = tilt.getEncoder();

    // Set our local max speeds
    winchMaxSpeed = ConstantsValues.winchMaxSpeed;
    winchMaxVolts = ConstantsValues.winchMaxVolts;
    tiltMaxSpeed = ConstantsValues.climberTiltMaxSpeed;
    tiltMaxVolts = ConstantsValues.climberTiltMaxVolts;


  }

  /**
   * Set the winch to a certain speed Note, this speed is clamped by the max value outlined in constants.
   * @param speed
   */
  public void setWinchSpeed(double speed) {
    winchLeader.set(MathUtil.clamp(speed, -winchMaxSpeed, winchMaxSpeed));
  }
  
  /**
   * Set the winch to a certain voltage Note, this voltage is clamped by the max value outlined in constants.
   * @param volts
   */
  public void setWinchVoltage(double volts) {
    winchLeader.setVoltage(MathUtil.clamp(volts, -winchMaxVolts, winchMaxVolts));
  }

  /**
   * Set the tilt to a certain speed Note, this speed is clamped by the max value outlined in constants.
   * @param speed
   */
  public void setTiltSpeed(double speed) {
    tilt.set(MathUtil.clamp(speed, -tiltMaxSpeed, tiltMaxSpeed));
  }

  /**
   * Set the tilt to a certain voltage. Note, this voltage is clamped by the max value outlined in constants.
   * @param volts
   */
  public void setTiltVoltage(double volts) {
    tilt.setVoltage(MathUtil.clamp(volts, -tiltMaxVolts, tiltMaxVolts));
  }

  /**
   * Stop the winch motors
   */
  public void stopWinch() {
    winchLeader.stopMotor();
  }

  /**
   * Stop the tilt motor
   */
  public void stopTilt() {
    tilt.stopMotor();
  }

  /**
   * Stop both the winch and the tilt motors
   */
  public void stopAll() {
    winchLeader.stopMotor();
    tilt.stopMotor();
  }

  @Override
  public void periodic() {
    
  }
}
