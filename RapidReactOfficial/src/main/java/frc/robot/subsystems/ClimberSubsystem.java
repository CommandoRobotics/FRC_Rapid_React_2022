// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;

public class ClimberSubsystem extends SubsystemBase {

  CANSparkMax winchLeader, winchFollower, tilt;
  RelativeEncoder winchEncoder, tiltEncoder;
  
  DigitalInput tiltForward, tiltBackward;

  NetworkTableInstance ntInst;

  public ClimberSubsystem() {

    // Instantiate motors controllers
    winchLeader = new CANSparkMax(ConstantsPorts.winchLeaderId, MotorType.kBrushless);
    winchFollower = new CANSparkMax(ConstantsPorts.winchFollowerId, MotorType.kBrushless);
    tilt = new CANSparkMax(ConstantsPorts.climbTiltId, MotorType.kBrushless);

    winchFollower.follow(winchLeader, false);

    // Instantiate encoders
    winchEncoder = winchLeader.getEncoder();
    tiltEncoder = tilt.getEncoder();

    // Set conversion factors
    winchEncoder.setPositionConversionFactor(ConstantsValues.climbWinchConversionFactor);
    tiltEncoder.setPositionConversionFactor(ConstantsValues.climbTiltConversionFactor);

    // Set soft limits
    winchLeader.setSoftLimit(SoftLimitDirection.kForward, ConstantsValues.climbWinchSoftLimit);
    
    // Enable soft limits
    enableWinchSoftLimits();

    // Instantiate limit switches
    tiltForward = new DigitalInput(ConstantsPorts.tiltForwardId);
    tiltBackward = new DigitalInput(ConstantsPorts.tiltBackwardId);

    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(winchFollower, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(winchLeader, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(tilt, DCMotor.getNEO(1));
    }

  }

  /**
   * Set the power of the winch
   * @param input 
   */
  public void setWinchVoltageFromController(double input) {
    input = MathUtil.applyDeadband(input, ConstantsValues.climbWinchDeadband)*ConstantsValues.climbWinchMaxVolts;
    winchLeader.setVoltage(input);
  }

  /**
   * Set the power of the tilt motor
   * @param input
   */
  public void setTiltVoltageFromController(double input) {
    input = MathUtil.applyDeadband(input, ConstantsValues.climbTiltDeadband)*ConstantsValues.climbTiltMaxVolts;
    if(input > 0) {
      if(tiltForward.get()) {
        tilt.setVoltage(0);
      } else {
        tilt.setVoltage(input);
      }
    } else {
      if(tiltBackward.get()) {
        tilt.setVoltage(0);
      } else {
        tilt.setVoltage(input);
      }
    }
  }

  /**
   * Set the power of the tilt motor WITHOUT using limit switches to limit position
   * @param power
   */
  public void setTiltNoLimitsFromController(double input) {
    tilt.setVoltage(MathUtil.applyDeadband(input, ConstantsValues.climbTiltDeadband)*ConstantsValues.climbTiltMaxVolts);
  }

  /**
   * Get the position of the tilt motor in rotations
   * @return
   */
  public double getTiltPosition() {
    return tiltEncoder.getPosition();
  }

  /**
   * Get the position of the winch motor in rotations
   * @return
   */
  public double getWinchPosition() {
    return winchEncoder.getPosition();
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
   * Reset the tilt encoder
   */
  public void resetTiltEncoder() {
    tiltEncoder.setPosition(0);
  }

  /**
   * Reset the winch encoder
   */
  public void resetWinchEncoder() {
    winchEncoder.setPosition(0);
  }

  /**
   * Disable the winch soft limits
   */
  public void disableWinchSoftLimit() {
    winchFollower.enableSoftLimit(SoftLimitDirection.kForward, false);
  }

  /**
   * Disable the winch soft limits
   */
  public void enableWinchSoftLimits() {
    winchFollower.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climbTiltEnc", getTiltPosition());
    SmartDashboard.putNumber("climbWinchEnc", getWinchPosition());
    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().run();
    }
  }
}
