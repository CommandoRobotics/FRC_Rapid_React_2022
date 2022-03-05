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

public class ClimberSubsystem extends SubsystemBase {

  Solenoid traversalClimb;
  DoubleSolenoid midClimb;
  CANSparkMax leftWinch, rightWinch;
  
  DigitalInput toplimitSwitch = new DigitalInput(ConstantsPorts.topLimitSwitchPort);
  DigitalInput bottomlimitSwitch = new DigitalInput(ConstantsPorts.bottomLimitSwitchPort);

  NetworkTableInstance ntInst;
  boolean previousMidState = false;
  boolean previousTraversalState = false;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    midClimb = new DoubleSolenoid(PneumaticsModuleType.REVPH, ConstantsPorts.midClimberForwardId, ConstantsPorts.midClimberReverseId);
    //traversalClimb = new Solenoid(PneumaticsModuleType.REVPH, ConstantsPorts.traversalClimberId);

    //leftWinch = new CANSparkMax(ConstantsPorts.leftWinchId, MotorType.kBrushless);
    //rightWinch = new CANSparkMax(ConstantsPorts.rightWinchId, MotorType.kBrushless);

    ntInst = NetworkTableInstance.getDefault();
    ntInst.getTable("CommandoDash").getSubTable("SensorData")
      .getEntry("midSolenoidState").setBoolean(isMidClimbExtended());
    // ntInst.getTable("CommandoDash").getSubTable("SensorData")
    //   .getEntry("traversalSolenoidState").setBoolean(traversalClimb.get());

  }

  //Extend Middle Solenoid
  public void midUp() {
    midClimb.set(Value.kForward);
  }

  //Retract Middle Solenoid
  public void midDown() {
    midClimb.set(Value.kReverse);
  }

  //Toggle Middle Solenoid 
  public void toggleMid() {
    midClimb.toggle();
  }

  //Extend Traversal Solenoid
  public void traversalUp() {
    traversalClimb.set(true);
  }

  //Retract Traversal Solenoid
  public void traversalDown() {
    traversalClimb.set(false);
  }

  //Toggle Traversal Solenoid
  public void toggleTraversal() {
    traversalClimb.toggle();
  }
  //Set the Power of the Right Winch
  public void setPowerRightWinch(double power) {
    if(power < 0) {
      if (bottomlimitSwitch.get()) {
        rightWinch.set(0);
      } else {
        rightWinch.set(power);
      }
    }
  }
  
  //Set the Power of the Left Winch
  public void setPowerLeftWinch(double power) {
    if(power > 0) {
      if (bottomlimitSwitch.get()) {
        leftWinch.set(0);
      } else {
        leftWinch.set(power);
      }
    }
  }

  /**
   * Get whether the mid climb is extended
   * @return
   */
  public boolean isMidClimbExtended() {
    return midClimb.get() == Value.kForward;
  }

  @Override
  public void periodic() {
    //Update CDD with the mid solenoid state
    boolean currMidState = isMidClimbExtended();
    if (previousMidState != currMidState) {
        ntInst.getTable("CommandoDash").getSubTable("SensorData")
            .getEntry("midSolenoidState").setBoolean(currMidState);
    }
    previousMidState = currMidState;

    //Update CDD with the traversal solenoid state
    // boolean currTraversalState = traversalClimb.get();
    // if (previousTraversalState != currTraversalState) {
    //     ntInst.getTable("CommandoDash").getSubTable("SensorData")
    //         .getEntry("traversalSolenoidState").setBoolean(currTraversalState);
    // }
    // previousTraversalState = currTraversalState;
  }
}
