// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsPorts;

public class ClimberSubsystem extends SubsystemBase {

  Solenoid midClimb, traversalClimb;
  CANSparkMax leftWinch, rightWinch;
  
  DigitalInput toplimitSwitch = new DigitalInput(2);
  DigitalInput bottomlimitSwitch = new DigitalInput(3);
  
  //TODO add Limit Switches


  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    midClimb = new Solenoid(PneumaticsModuleType.REVPH, ConstantsPorts.midClimberId);
    traversalClimb = new Solenoid(PneumaticsModuleType.REVPH, ConstantsPorts.traversalClimberId);

    leftWinch = new CANSparkMax(ConstantsPorts.leftWinchId, MotorType.kBrushless);
    rightWinch = new CANSparkMax(ConstantsPorts.rightWinchId, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per schedule run 
  }
   //Extend Middle Solenoid
   public void midUp() {
    midClimb.set(true);
  }

  //Retract Middle Solenoid
  public void midDown() {
    midClimb.set(false);
  }

  //Toggle Middle Solenoid 
  public void toggleMid() {
    midClimb.toggle();

  }

  //Extend Traversal Solenoid
  public void traversalUp() {
    midClimb.set(true);
  }

  //Retract Traversal Solenoid
  public void traversalDown() {
    midClimb.set(false);
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

  

}
