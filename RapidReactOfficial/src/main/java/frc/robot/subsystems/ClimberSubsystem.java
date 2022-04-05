// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // Instantiate limit switches
    tiltForward = new DigitalInput(ConstantsPorts.tiltForwardId);
    tiltBackward = new DigitalInput(ConstantsPorts.tiltBackwardId);

  }

  /**
   * Set the power of the winch
   * @param power 
   */
  public void setWinch(double power) {
    winchLeader.set(MathUtil.clamp(MathUtil.applyDeadband(power, ConstantsValues.climbWinchDeadband), -ConstantsValues.climbWinchMaxSpeed, ConstantsValues.climbWinchMaxSpeed));
  }

  /**
   * Set the power of the tilt motor
   * @param power
   */
  public void setTilt(double power) {
    power = MathUtil.clamp(MathUtil.applyDeadband(power, ConstantsValues.climbTiltDeadband), -ConstantsValues.climbTiltMaxSpeed, ConstantsValues.climbTiltMaxSpeed);
    if(power > 0) {
      if(tiltForward.get()) {
        tilt.set(0);
      } else {
        tilt.set(power);
      }
    } else {
      if(tiltBackward.get()) {
        tilt.set(0);
      } else {
        tilt.set(power);
      }
    }
  }

  /**
   * Set the power of the tilt motor WITHOUT using limit switches to limit position
   * @param power
   */
  public void setTiltNoLimits(double power) {
    tilt.set(MathUtil.clamp(MathUtil.applyDeadband(power, ConstantsValues.climbTiltDeadband), -ConstantsValues.climbTiltMaxSpeed, ConstantsValues.climbTiltMaxSpeed));
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

  @Override
  public void periodic() {
  }
}
