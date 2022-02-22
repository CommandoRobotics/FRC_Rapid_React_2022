// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;

public class IndexSubsystem extends SubsystemBase {

  int[] blocks = {0,0,0,0,0,0};

  CANSparkMax ramp, vertical, transferLeader, transferFollower;
  RelativeEncoder rampEncoder, verticalEncoder;

  AnalogInput verticalSensor, rampSensor, entranceSensor;

  // What index of the block array refers to what block section
  private final int entranceSensorBlock = 0;
  private final int entranceToRampBlock = 1;
  private final int rampSensorBlock = 2;
  private final int rampToVerticalBlock = 3;
  private final int verticalSensorBlock = 4;
  private final int verticalToShooterBlock = 5;

  boolean verticalSensorTriggeredPrevious, rampSensorTriggeredPrevious, entranceSensorTriggeredPrevious;
  long previousBlockUpdateTimeMillis;

  public IndexSubsystem() {
    // Instantiate Spark Maxes
    ramp = new CANSparkMax(ConstantsPorts.rampId, MotorType.kBrushless);
    vertical = new CANSparkMax(ConstantsPorts.verticalId, MotorType.kBrushless);
    transferLeader = new CANSparkMax(ConstantsPorts.transferLeaderId, MotorType.kBrushless);
    transferFollower = new CANSparkMax(ConstantsPorts.transferFollowerId, MotorType.kBrushless);

    ramp.setInverted(false);
    vertical.setInverted(false);
    transferLeader.setInverted(false);

    // Follow and set inversion
    transferFollower.follow(transferLeader, true);

    rampEncoder = ramp.getEncoder();
    verticalEncoder = vertical.getEncoder();

    rampEncoder.setPositionConversionFactor(ConstantsValues.rampPositionConversionFactor);
    verticalEncoder.setPositionConversionFactor(ConstantsValues.verticalPositionConversionFactor);
    rampEncoder.setVelocityConversionFactor(ConstantsValues.rampVelocityConversionFactor);
    verticalEncoder.setVelocityConversionFactor(ConstantsValues.verticalVelocityConversionFactor);

    verticalSensor = new AnalogInput(ConstantsPorts.verticalIndexSensorPort);
    rampSensor = new AnalogInput(ConstantsPorts.rampIndexSensorPort);
    entranceSensor = new AnalogInput(ConstantsPorts.indexEntranceSensorPort);

    // Set the index sensors initial state based on whether a ball is present
    verticalSensorTriggeredPrevious = isVerticalSensorTriggered();
    rampSensorTriggeredPrevious = isRampSensorTriggered();
    entranceSensorTriggeredPrevious = isEntranceSensorTriggered();
    if(verticalSensorTriggeredPrevious) {
      blocks[verticalSensorBlock] = 1;
    }
    if(rampSensorTriggeredPrevious) {
      blocks[rampSensorBlock] = 1;
    }
    if(entranceSensorTriggeredPrevious) {
      blocks[entranceSensorBlock] = 1;
    }

    previousBlockUpdateTimeMillis = System.currentTimeMillis();

    // Add motors to the simulation
    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(ramp, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(vertical, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(transferLeader, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(transferFollower, DCMotor.getNeo550(1));
    }
  }

  /**
   * Set the speed of the ramp index motor
   * @param speed
   */
  public void setRamp(double speed) {
    ramp.set(speed);
  }

  /**
   * Set the speed of the vertical index motor
   * @param speed
   */
  public void setVertical(double speed) {
    vertical.set(speed);
  }

  /**
   * Set the speed of the transfer motors
   * @param speed
   */
  public void setTransfer(double speed) {
    transferLeader.set(speed);
  }

  /**
   * Set the voltage of the ramp index motor
   * @param volts
   */
  public void setRampVoltage(double volts) {
    ramp.setVoltage(volts);
  }

  /**
   * Set the voltage of the vertical index motor
   * @param volts
   */
  public void setVerticalVoltage(double volts) {
    vertical.setVoltage(volts);
  }

  /**
   * Set the voltage of the transfer motors
   * @param volts
   */
  public void setTransferVoltage(double volts) {
    transferLeader.setVoltage(volts);
  }

  /**
   * Stop the ramp index motor
   */
  public void stopRamp() {
    ramp.set(0);
  }

  /**
   * Stop the vertical index motor
   */
  public void stopVertical() {
    vertical.set(0);
  }

  /**
   * Stop the transfer index motors
   */
  public void stopTransfer() {
    transferLeader.set(0);
  }

  /**
   * Stop all of the index motors
   */
  public void stopAll() {
    ramp.set(0);
    vertical.set(0);
    transferLeader.set(0);
  }

  /**
   * Get the position of the ramp motor
   * @return 
   */
  public double getRampPosition() {
    return rampEncoder.getPosition();
  }

  /**
   * Get the position of the vertical motor
   * @return
   */
  public double getVerticalPosition() {
    return verticalEncoder.getPosition();
  }

  /**
   * Get the velocity of the ramp motor
   * @return 
   */
  public double getRampVelocity() {
    return rampEncoder.getVelocity();
  }

  /**
   * Get the velocity of the vertical motor
   * @return
   */
  public double getVerticalVelocity() {
    return verticalEncoder.getVelocity();
  }

  /**
   * Reset the ramp encoder
   */
  public void resetRampEncoder() {
    rampEncoder.setPosition(0);
  }

  /**
   * Reset the vertical encoder
   */
  public void resetVerticalEncoder() {
    verticalEncoder.setPosition(0);
  }

  /**
   * Check if the entrance sensor is currently triggered
   * @return
   */
  public boolean isEntranceSensorTriggered() {
    return entranceSensor.getVoltage() > ConstantsValues.indexSensorThreshold;
  }

  /**
   * Check if the ramp sensor is currently triggered
   * @return
   */
  public boolean isRampSensorTriggered() {
    return rampSensor.getVoltage() > ConstantsValues.indexSensorThreshold;

  }

  /**
   * Check if the vertical sensor is currently triggered
   * @return
   */
  public boolean isVerticalSensorTriggered() {
    return verticalSensor.getVoltage() > ConstantsValues.indexSensorThreshold;
  }

  /**
   * Update the block system. This should be called in the periodic method for this subsystem.
   */
  public void updateBlocks() {
    int[] newBlocks = blocks;
    boolean entranceTriggered = isEntranceSensorTriggered();
    boolean rampTriggered = isRampSensorTriggered();
    boolean verticalTriggered = isVerticalSensorTriggered();

    // Update shooter block
    long timeElapsedMillis = System.currentTimeMillis()-previousBlockUpdateTimeMillis;
    if(timeElapsedMillis >= ConstantsValues.shooterBlockTime && blocks[verticalToShooterBlock] == 1) {
      newBlocks[verticalToShooterBlock] = 0;
    }

    // Update blocks based on entrance sensor
    if(entranceTriggered) {
      if(!entranceSensorTriggeredPrevious) {
        newBlocks[entranceSensorBlock] = 1;
      }
    } else {
      if(entranceSensorTriggeredPrevious) {
        //TODO add motor direction based logic
        // If running forwards, add 1 to the entrance to index block.
        // If running backwards, remove the ball from the index.
      }
    }    

    // Update blocks based on ramp sensor
    if(rampTriggered) {
      if(!rampSensorTriggeredPrevious) {
        newBlocks[rampSensorBlock] = 1;
        if(getRampVelocity() > 0) {
          newBlocks[entranceToRampBlock] = 0;
        } else if(getRampVelocity() < 0) {
          newBlocks[rampToVerticalBlock] = 0;
        }
      }
    } else {
      if(rampSensorTriggeredPrevious) {
        newBlocks[rampSensorBlock] = 0;
        if(getRampVelocity() > 0) {
          newBlocks[rampToVerticalBlock] = 1;
        } else if(getRampVelocity() < 0) {
          newBlocks[entranceToRampBlock] = 1;
        }
      }
    }

    // Update blocks based on vertical sensor
    if(verticalTriggered) {
      if(!verticalSensorTriggeredPrevious) {
        newBlocks[verticalSensorBlock] = 1;
        if(getVerticalVelocity() > 0) {
          newBlocks[rampToVerticalBlock] = 0;
        } 
      }
    } else {
      if(verticalSensorTriggeredPrevious) {
        newBlocks[rampSensorBlock] = 0;
        if(getVerticalVelocity() > 0) {
          newBlocks[verticalToShooterBlock] = 1;
        } else if(getVerticalVelocity() < 0) {
          newBlocks[rampToVerticalBlock] = 1;
        }
      }
    }

    // Update variables for next iteration
    blocks = newBlocks;
    entranceSensorTriggeredPrevious = entranceTriggered;
    rampSensorTriggeredPrevious = rampTriggered;
    verticalSensorTriggeredPrevious = verticalTriggered;
    previousBlockUpdateTimeMillis = System.currentTimeMillis();

  }

  /**
   * Get whether a ball is in the entrance sensor block
   * @return
   */
  public boolean isBallAtEntrance() {
    return blocks[entranceSensorBlock] > 0;
  }

  /**
   * Get whether a ball is in the entrance sensor to ramp sensor block
   * @return
   */
  public boolean isBallBetweenEntranceAndRamp() {
    return blocks[entranceToRampBlock] > 0;
  }

  /**
   * Get whether a ball is in the ramp sensor block
   * @return
   */
  public boolean isBallAtRamp() {
    return blocks[rampSensorBlock] > 0;
  }
  
  /**
   * Get whether a ball is in the ramp sensor to vertical sensor block
   * @return
   */
  public boolean isBallBetweenRampAndVertical() {
    return blocks[rampToVerticalBlock] > 0;
  }

  /**
   * Get whether a ball is in vertical sensor block
   * @return
   */
  public boolean isBallAtVertical() {
    return blocks[verticalSensorBlock] > 0;
  }

  /**
   * Get whether a ball is in vertical sensor to shooter block
   * @return
   */
  public boolean isBallInShooter() {
    return blocks[verticalToShooterBlock] > 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateBlocks();
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
