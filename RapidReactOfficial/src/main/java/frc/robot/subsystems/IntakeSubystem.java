// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsPorts;

public class IntakeSubystem extends SubsystemBase {

  CANSparkMax intake;
  RelativeEncoder intakeEncoder;
  Solenoid lifter;

  public IntakeSubystem() {
      intake = new CANSparkMax(ConstantsPorts.intakeID, MotorType.kBrushless);

      intakeEncoder = intake.getEncoder();

      lifter = new Solenoid(PneumaticsModuleType.REVPH, ConstantsPorts.lifterID);
      
  }

  //running

  //set power
  public void setPower(double power) {
      intake.set(power);
  }
  //set volatage
  public void setVoltage(double Volts) {
      intake.setVoltage(Volts);
      
  }
  //stop
  public void stop() {
      intake.stopMotor();
  }
  //intakeIn()
  public void IntakeIn() {
     intake.set(ConstantsPorts.IntakePower);
  }
  //intakeOut()
  public void IntakeOut() {
      intake.set(ConstantsPorts.EjectPower);
  }
  //lifting/solenoid stuff

  //raise lifter
  boolean on = true;

  public void raiseLifter() {
      boolean on = true;
      lifter.set(on);
  }

  //lower lifter

  public void lowerLifter() {
      boolean on = false;
      lifter.set(on);
  }
  //toggle
  public void toggleLifter() {       
      if ( on = true) {
          lowerLifter();

      } else {
          raiseLifter();
      }
  }
  //sensors

  //get velocity
public void getVelocity() {
  intakeEncoder.getVelocity();
}
public void resetEncoder() {
  intakeEncoder.setPosition(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
