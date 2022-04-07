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


  //Winch


  /**
   * Sets the speed of the winch motors in a range of 0 to 1
   */
  public void setWinchSpeed(double speed) {
    winchLeader.set(speed);
  }

  /**
   * Sets the voltage for the winch motors
   * @param volts
   */
  public void setWinchVoltage(double volts) {
    winchLeader.setVoltage(volts);
  }

  /**
   * Stops the winch motors
   */
  public void stopWinch() {
    winchLeader.setVoltage(0);
    winchLeader.stopMotor();
  }

  /**
   * Sets the speed of the winch motors but will stop the motors using encoder values 
   * if they try to go too far. This method has an upper limit but not a lower limit
   * @param speed
   */
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

  /**
   * Sets the voltage of the winch motors but will stop the motors using encoder values 
   * if they try to go too far. This method has an upper limit but not a lower limit
   * @param volts
   */
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


  /**
   * Sets the speed of the tilt motor from 0 to 1
   * @param speed
   */
  public void setTiltSpeed(double speed) {
    tilt.set(speed);
  }

  /**
   * Sets the voltage of the tilt motor
   * @param volts
   */
  public void setTiltVoltage(double volts) {
    tilt.setVoltage(volts);
  }

  /**
   * Stops the tilt motor
   */
  public void stopTilt() {
    tilt.setVoltage(0);
    tilt.stopMotor();
  }

  /**
   * Sets the voltage of the tilt motor but uses software (encoder) or limit switches
   * to limit how far the motor goes. If the motor is trying to past a limit, this method will 
   * instead stop powering the motor. The tilt has both forward and backward limits.
   * @param volts
   * @param useLimitSwitches whether to use limit switches as the limits. If this is false, it will
   *                         default to encoder values as the limits
   */
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

  /**
   * Sets the speed of the tilt motor but uses software (encoder) or limit switches
   * to limit how far the motor goes. If the motor is trying to past a limit, this method will 
   * instead stop powering the motor. The tilt has both forward and backward limits.
   * @param speed
   * @param useLimitSwitches whether to use limit switches as the limits. If this is false, it will
   *                         default to encoder values as the limits
   */
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

  /**
   * Uses a mix of limit switches and encoder values to limit the speed of the tilt motor. This uses
   * a limit switch as the forward limit for the tilt and encoder values as the reverse limit 
   * 
   * </p>It would be useful to also have a way to reset the the tilt encoder when the limit switch is hit.
   * Or you could implement it into this method later
   * @param speed
   */
  public void setTiltSpeedWithLimitsMixed(double speed) {
    if (speed > 0) {
      if (tiltForward.get()) {
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

  /**
   * Uses a mix of limit switches and encoder values to limit the voltage of the tilt motor. This uses
   * a limit switch as the forward limit for the tilt and encoder values as the reverse limit<p>
   * 
   * </p>It would be useful to also have a way to reset the the tilt encoder when the limit switch is hit.
   * Or you could implement it into this method later
   * @param volts
   */
  public void setTiltVolatgeWithLimitsMixed(double volts) {
    if (volts > 0) {
      if (tiltForward.get()) {
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


  //Sensors


  /**
   * Gets the current number of rotations of the winch motor
   * @return
   */
  public double getWinchRotations() {
    return winchEncoder.getPosition();
  }

  /**
   * Gets the current angle of the tilt motor in degrees
   * @return
   */
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

    //Update CommandoDash
    NetworkTableInstance.getDefault().getTable("CommandoDash").getSubTable("SensorData").getEntry("tiltAngle").setDouble(getTiltAngle());

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
