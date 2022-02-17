// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;

public class IndexSubsystem extends SubsystemBase {

  CANSparkMax horizontal, vertical, transferLeader, transferFollower;
  RelativeEncoder horizontalEncoder, verticalEncoder;

  public IndexSubsystem() {
    // Instantiate Spark Maxes
    horizontal = new CANSparkMax(ConstantsPorts.horizontalId, MotorType.kBrushless);
    vertical = new CANSparkMax(ConstantsPorts.verticalId, MotorType.kBrushless);
    transferLeader = new CANSparkMax(ConstantsPorts.transferLeaderId, MotorType.kBrushless);
    transferFollower = new CANSparkMax(ConstantsPorts.transferFollowerId, MotorType.kBrushless);

    horizontal.setInverted(false);
    vertical.setInverted(false);
    transferLeader.setInverted(false);

    // Follow and set inversion
    transferFollower.follow(transferLeader, true);

    horizontalEncoder = horizontal.getEncoder();
    verticalEncoder = vertical.getEncoder();

    horizontalEncoder.setPositionConversionFactor(ConstantsValues.horizontalPositionConversionFactor);
    verticalEncoder.setPositionConversionFactor(ConstantsValues.verticalPositionConversionFactor);
    horizontalEncoder.setVelocityConversionFactor(ConstantsValues.horizontalVelocityConversionFactor);
    verticalEncoder.setVelocityConversionFactor(ConstantsValues.verticalVelocityConversionFactor);


    // Add motors to the simulation
    if(Robot.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(horizontal, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(vertical, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(transferLeader, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(transferFollower, DCMotor.getNeo550(1));
    }
  }

  /**
   * Set the speed of the horizontal index motor
   * @param speed
   */
  public void setHorizontal(double speed) {
    horizontal.set(speed);
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
   * Set the voltage of the horizontal index motor
   * @param volts
   */
  public void setHorizontalVoltage(double volts) {
    horizontal.setVoltage(volts);
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
   * Stop the horizontal index motor
   */
  public void stopHorizontal() {
    horizontal.set(0);
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
    horizontal.set(0);
    vertical.set(0);
    transferLeader.set(0);
  }

  /**
   * Get the position of the horizontal motor
   * @return 
   */
  public double getHorizontalPosition() {
    return horizontalEncoder.getPosition();
  }

  /**
   * Get the position of the vertical motor
   * @return
   */
  public double getVerticalPosition() {
    return verticalEncoder.getPosition();
  }

  /**
   * Get the velocity of the horizontal motor
   * @return 
   */
  public double getHorizontalVelocity() {
    return horizontalEncoder.getVelocity();
  }

  /**
   * Get the velocity of the vertical motor
   * @return
   */
  public double getVerticalVelocity() {
    return verticalEncoder.getVelocity();
  }

  /**
   * Reset the horizontal encoder
   */
  public void resetHorizontalEncoder() {
    horizontalEncoder.setPosition(0);
  }

  /**
   * Reset the vertical encoder
   */
  public void resetVerticalEncoder() {
    verticalEncoder.setPosition(0);
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
