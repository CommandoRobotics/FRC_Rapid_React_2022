// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsPorts;

public class ClimberSubsystem extends SubsystemBase {

  // Motor controllers
  CANSparkMax winchLeader, winchFollower, tilt;

  // Encoders
  RelativeEncoder winchEncoder, tiltEncoder;
  

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

  }

  /**
   * Set the winch to a certain speed
   * @param speed
   */
  public void setWinchSpeed(double speed) {
    winchLeader.set(speed);
  }
  
  /**
   * Set the winch to a certain voltage
   * @param volts
   */
  public void setWinchVoltage(double volts) {
    winchLeader.setVoltage(volts);
  }

  @Override
  public void periodic() {
    
  }
}
