// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.channels.NotYetBoundException;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HoundCargo extends CommandBase {

  DoubleSupplier x,y,rotation;
  ProfiledPIDController rotationPID;
  PIDController xPID, yPID;
  DriveSubsystem driveSubsystem;
  IntakeSubsystem intakeSubsystem;
  NetworkTableInstance ntInst;
  NetworkTable houndTable;

  /** Creates a new HoundCargo. */
  public HoundCargo(IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, 
    DoubleSupplier y, DoubleSupplier x, DoubleSupplier rotation) {
    this.intakeSubsystem = intakeSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    this.rotation = rotation;
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

    if (Robot.isSimulation()) {
    ntInst = NetworkTableInstance.getDefault();
    ntInst.stopServer();
    ntInst.stopClient();
    ntInst.stopDSClient();
    ntInst.setServer("10.58.89.12");
    ntInst.startClient();
    houndTable = ntInst.getTable("photonvision").getSubTable("CargoHound");
    }

    addRequirements(intakeSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPID.calculate(0);
    yPID.calculate(0);
    rotationPID.calculate(0);
    driveSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult houndResult = intakeSubsystem.getHoundData();
    if (houndResult.hasTargets()) {
      driveSubsystem.driveMecanum(
        yPID.calculate(-intakeSubsystem.getDistanceToCargo(houndResult)), 
        xPID.calculate(houndResult.getBestTarget().getYaw()),
        rotationPID.calculate(-houndResult.getBestTarget().getYaw()),
        false);
        SmartDashboard.putNumber("HoundDis", intakeSubsystem.getDistanceToCargo(houndResult));
    } else {
      driveSubsystem.driveMecanum(y.getAsDouble(), x.getAsDouble(), rotation.getAsDouble(), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
