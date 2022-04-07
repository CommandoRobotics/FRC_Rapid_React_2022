// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

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

    tilt.setInverted(true);

    // Instantiate encoders
    winchEncoder = winchLeader.getEncoder();
    tiltEncoder = tilt.getEncoder();

    // Set conversion factors
    winchEncoder.setPositionConversionFactor(ConstantsValues.climbWinchConversionFactor);
    tiltEncoder.setPositionConversionFactor(ConstantsValues.climbTiltConversionFactor);

    // Put soft limits on dashboard
    SmartDashboard.putNumber("climbWinchLimit", ConstantsValues.winchHeightLimitRotations);
    SmartDashboard.putNumber("tiltForwardLimit", ConstantsValues.tiltForwardLimit);
    SmartDashboard.putNumber("tiltReverseLimit", ConstantsValues.tiltReverseLimit);

    // Instantiate limit switches
    tiltForward = new DigitalInput(ConstantsPorts.tiltForwardId);
    tiltBackward = new DigitalInput(ConstantsPorts.tiltBackwardId);

    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(winchFollower, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(winchLeader, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(tilt, DCMotor.getNEO(1));
    }

  }

  //CLIMB METHODS

  //Winch
  //setSpeed
  public void setWinchSpeed(double speed) {
    winchLeader.set(speed);
  }

  //setVoltage
  public void setWinchVoltage(double volts) {
    winchLeader.setVoltage(volts);
  }

  //stopWinch
  public void stopWinch() {
    winchLeader.stopMotor();
    winchLeader.setVoltage(0);
  }

  //setVoltsLimits soft limits
  public void setWinchSpeedWithLimits(double speed) {
    if (speed > 0) {
      if (getWinchRotations() >= ConstantsValues.winchHeightLimitRotations) {
        setWinchSpeed(0);
        return;
      } else {
        setWinchSpeed(speed);
      }
    } else {
      setWinchSpeed(speed);
    }
  }

  //setVoltsLimits soft limits
  public void setWinchVoltageWithLimits(double volts) {
    if (volts > 0) {
      if (getWinchRotations() >= ConstantsValues.winchHeightLimitRotations) {
        setWinchVoltage(0);
        return;
      } else {
        setWinchVoltage(volts);
      }
    } else {
      setWinchVoltage(volts);
    }
  }

  //Tilt
  //setSpeed
  public void setTiltSpeed(double speed) {
    tilt.set(speed);
  }

  //setVoltage
  public void setTiltVoltage(double volts) {
    tilt.setVoltage(volts);
  }

  //stopTilt
  public void stopTilt() {
    tilt.stopMotor();
    tilt.setVoltage(0);
  }

  //setVoltageLimits
  //TODO FORWARD MUST BE TOWARDS THE FRONT OF THE ROBOT
  public void setTiltVoltageWithLimits(double volts, boolean useLimitSwitches) {
    if(useLimitSwitches) {
      if (volts > 0) {
        if (tiltForward.get()) {
          setTiltVoltage(0);
          return;
        } else {
          setTiltVoltage(volts);
        }
      } else {
        if (tiltBackward.get()) {
          setTiltVoltage(0);
          return;
        } else {
          setTiltVoltage(volts);
        }
      }
    } else {
      if (volts > 0) {
        if (getTiltAngle() >= ConstantsValues.tiltForwardLimit) {
          setTiltVoltage(0);
          return;
        } else {
          setTiltVoltage(volts);
        }
      } else {
        if (getTiltAngle() <= ConstantsValues.tiltReverseLimit) {
          setTiltVoltage(0);
          return;
        } else {
          setTiltVoltage(volts);
        }
      }
    }
  }

  //setSpeedLimits
  public void setTiltSpeedWithLimits(double speed, boolean useLimitSwitches) {
    if(useLimitSwitches) {
      if (speed > 0) {
        if (tiltForward.get()) {
          setTiltSpeed(0);
          return;
        } else {
          setTiltSpeed(speed);
        }
      } else {
        if (tiltBackward.get()) {
          setTiltSpeed(0);
          return;
        } else {
          setTiltSpeed(speed);
        }
      }
    } else {
      if (speed > 0) {
        if (getTiltAngle() >= ConstantsValues.tiltForwardLimit) {
          setTiltSpeed(0);
          return;
        } else {
          setTiltSpeed(speed);
        }
      } else {
        if (getTiltAngle() <= ConstantsValues.tiltReverseLimit) {
          setTiltSpeed(0);
          return;
        } else {
          setTiltSpeed(speed);
        }
      }
    }
  }

  //Sensors
  //getWinchHeight
  public double getWinchRotations() {
    return winchEncoder.getPosition();
  }

  //getTiltAngle
  public double getTiltAngle() {
    return tiltEncoder.getPosition();
  }

  /**
   * Reset the winch encoder
   */
  public void resetWinchEncoder() {
    winchEncoder.setPosition(0);
  }

  /**
   * Reset the tilt encoder
   */
  public void resetTiltEncoder() {
    tiltEncoder.setPosition(0);
  }



  //Periodic
  //Upate whether we need to reset encoders

  @Override
  public void periodic() {
    //Update Encoder values to SmartDash
    SmartDashboard.putNumber("climbTiltEnc", getTiltAngle());
    SmartDashboard.putNumber("climbWinchEnc", getWinchRotations());

    //Check if there are new Limit values from the SmartDash
    double currWinchLimit = SmartDashboard.getNumber("climbWinchLimit", ConstantsValues.winchHeightLimitRotations);
    if (currWinchLimit != ConstantsValues.winchHeightLimitRotations) {
      ConstantsValues.winchHeightLimitRotations = currWinchLimit;
    }
    double currTiltReverseLimit = SmartDashboard.getNumber("tiltReverseLimit", ConstantsValues.tiltReverseLimit);
    if (currTiltReverseLimit != ConstantsValues.tiltReverseLimit) {
      ConstantsValues.tiltReverseLimit = currTiltReverseLimit;
    }
    double currTiltForwardLimit = SmartDashboard.getNumber("tiltForwardLimit", ConstantsValues.tiltForwardLimit);
    if (currTiltForwardLimit != ConstantsValues.tiltForwardLimit) {
      ConstantsValues.tiltForwardLimit = currTiltForwardLimit;
    }

    //TODO if we have limit switch, uncomment this for auto encoder resetting 
    //MAY NOT DO THIS BC THE CLIMBER SHAKES
    // //Reset Tilt encoders if limit switches are active
    // if (tiltForward.get()) {
    //   resetTiltEncoder();
    // }
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
