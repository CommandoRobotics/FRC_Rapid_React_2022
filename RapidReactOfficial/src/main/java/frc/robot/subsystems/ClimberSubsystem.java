// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsPorts;

public class ClimberSubsystem extends SubsystemBase {

  Solenoid midClimb, traversalClimb;
  CANSparkMax leftWinch, rightWinch;
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
   public void MidUp() {
    boolean on = true;
    midClimb.set(on);
  }

  //Retract Middle Solenoid
  public void MidDown() {
    boolean on = false;
    midClimb.set(on);
  }
  //Toggle Middle Solenoid
  
  public void ToggleMid() {
    boolean on;
    if (on = true) {
      MidDown();
    } else {
      MidUp();
    }
  }
  //Extend Traversal Solenoid
  public void TraversalUp() {
    boolean on = true;
    midClimb.set(on);
  }
  //Retract Traversal Solenoid
  public void TraversalDown() {
    boolean on = false;
    midClimb.set(on);
  }
  //Toggle Traversal Solenoid
  public void ToggleTraversal() {
    boolean on;
    if (on = true) {
      TraversalDown();
    } else {
      TraversalUp();
    }
  }
  //Set the Power of the Right Winch
   
  
  //Set the Power of the Left Winch
  

}
