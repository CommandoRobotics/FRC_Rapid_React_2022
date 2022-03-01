// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HoundCargo extends CommandBase {

  DoubleSupplier x,y,rotation;
  ProfiledPIDController rotationPID;
  PIDController xPID, yPID;
  Timer noCargoTimer;
  PhotonPipelineResult previousResult;
  DriveSubsystem driveSubsystem;
  IntakeSubsystem intakeSubsystem;
  NetworkTableInstance ntInst;

  /** Creates a new HoundCargo. THIS COMMAND MUST BE INTERUPTED TO STOP */
  public HoundCargo(IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, 
    DoubleSupplier y, DoubleSupplier x, DoubleSupplier rotation) {
    this.intakeSubsystem = intakeSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    this.rotation = rotation;
    noCargoTimer = new Timer();
    xPID = new PIDController(
      ConstantsValues.houndXP, 
      ConstantsValues.houndXI,
      ConstantsValues.houndXD);
    yPID = new PIDController(
      ConstantsValues.houndYP, 
      ConstantsValues.houndYI,
      ConstantsValues.houndYD);
    rotationPID = new ProfiledPIDController(
      ConstantsValues.houndRP, 
      ConstantsValues.houndRI,
      ConstantsValues.houndRD,
      ConstantsValues.houndRConstraints);
    xPID.setSetpoint(0);
    yPID.setSetpoint(0);
    rotationPID.setGoal(0);
    SmartDashboard.putData(xPID);
    SmartDashboard.putData(yPID);
    SmartDashboard.putData(rotationPID);

    //Photon NT networking to use with an at-home simulation
    ntInst = NetworkTableInstance.getDefault();
    // if (Robot.isSimulation()) {
    // ntInst.stopServer();
    // ntInst.stopClient();
    // ntInst.stopDSClient();
    // ntInst.setServer("10.58.89.12");
    // ntInst.startClient();
    // }

    addRequirements(intakeSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPID.calculate(0);
    yPID.calculate(0);
    rotationPID.calculate(0);
    driveSubsystem.stop();
    previousResult = intakeSubsystem.getHoundData();
    ntInst.getTable("CommandoDash").getSubTable("SensorData").getEntry("isHounding").setBoolean(true);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get the current data from the CargoHound
    PhotonPipelineResult houndResult = intakeSubsystem.getHoundData();
    
    //Check if there are targets
    if (houndResult.hasTargets()) {
      //Drive based PID from the different values measured
      driveSubsystem.driveMecanum(
          IntakeSubsystem.scaleAroundZero(
            //TODO Not sure if this needs to be negative or not
            yPID.calculate(-intakeSubsystem.getDistanceToCargo(houndResult)), 
            ConstantsValues.minHoundPIDOut), 
          IntakeSubsystem.scaleAroundZero(
            xPID.calculate(-houndResult.getBestTarget().getYaw()),
            ConstantsValues.minHoundPIDOut),
          IntakeSubsystem.scaleAroundZero(
            rotationPID.calculate(-houndResult.getBestTarget().getYaw()),
            ConstantsValues.minHoundPIDOut),
        false);
      //Start the timer from the time we last saw the ball
      previousResult = houndResult;
      noCargoTimer.stop();
      noCargoTimer.reset();
      noCargoTimer.start();
    } else if ((noCargoTimer.get() <= ConstantsValues.noCargoTime) && (previousResult != null) && previousResult.hasTargets()) {
      //If we've seen a ball within the alloted time, but don't see one now, use a previous input to continue following the ball
      driveSubsystem.driveMecanum(
        IntakeSubsystem.scaleAroundZero(
          //TODO Not sure if this needs to be negative or not
          yPID.calculate(-intakeSubsystem.getDistanceToCargo(previousResult)), 
          ConstantsValues.minHoundPIDOut), 
        IntakeSubsystem.scaleAroundZero(
          xPID.calculate(-previousResult.getBestTarget().getYaw()),
          ConstantsValues.minHoundPIDOut),
        IntakeSubsystem.scaleAroundZero(
          rotationPID.calculate(-previousResult.getBestTarget().getYaw()),
          ConstantsValues.minHoundPIDOut),
        false);
    } else {
      //If no ball is found in the time alloted, default to driver control
      driveSubsystem.driveMecanum(y.getAsDouble(), x.getAsDouble(), rotation.getAsDouble(), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ntInst.getTable("CommandoDash").getSubTable("SensorData").getEntry("isHounding").setBoolean(false);
    xPID.calculate(0);
    yPID.calculate(0);
    rotationPID.calculate(0);
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
