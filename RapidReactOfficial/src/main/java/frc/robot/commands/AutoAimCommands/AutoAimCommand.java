// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAimCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConstantsValues;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoAimCommand extends CommandBase {

  DoubleSupplier x, y, rotation;
  DriveSubsystem driveSubsystem;
  AutoAimSubsystem autoAimSubsystem;
  NetworkTable sensorTable;

  /**
   * An AimDrivetrainUsingVision command
   * @param x The x input from the driver controller
   * @param y The y input from the driver controller
   * @param z The z input from the driver controller
   * @param driveSubsystem The drive subsystem
   * @param autoAimSubsystem The auto aim subsystem
   */
  public AutoAimCommand(DoubleSupplier y, DoubleSupplier x, DoubleSupplier rotation, DriveSubsystem driveSubsystem, AutoAimSubsystem autoAimSubsystem) {
    this.y = y;
    this.x = x;
    this.rotation = rotation;
    this.driveSubsystem = driveSubsystem;
    this.autoAimSubsystem = autoAimSubsystem;
    addRequirements(driveSubsystem);
    addRequirements(autoAimSubsystem);
    // Instantiate sensor table for CommandoDash integration
    sensorTable = NetworkTableInstance.getDefault().getTable("CommandoDash").getSubTable("SensorData");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoAimSubsystem.enableLimelightLed();
    sensorTable.getEntry("isAutoAiming").setBoolean(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean fieldCentric = driveSubsystem.isFieldCentricToggleEnabled();
    autoAimSubsystem.enableLimelightLed();
    if(autoAimSubsystem.isTargetSeen()) {
      // Automatically rotate the robot to the target

      // Calculate rotation value
      double rotationValue = -IntakeSubsystem.scaleAroundZero(
        autoAimSubsystem.calculatePanOutput(
          autoAimSubsystem.getLimelightXOffset()
        ), 
        ConstantsValues.panPidMinOutput
      );

      // Update CommandoDash
      sensorTable.getEntry("isRobotAimed").setBoolean(rotationValue == 0);

      // Drive the robot
      driveSubsystem.driveMecanum(
        // Forward/reverse value
        y.getAsDouble(), 
        // Strafe value
        x.getAsDouble(),
        rotationValue,
        fieldCentric
        );
    } else {
      // Drive based on driver commands since we can't see the target
      driveSubsystem.driveMecanum(
        y.getAsDouble(), 
        x.getAsDouble(), 
        rotation.getAsDouble(), 
        fieldCentric
        );
      // Update CommandoDash
      sensorTable.getEntry("isRobotAimed").setBoolean(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoAimSubsystem.disableLimelightLed();
    driveSubsystem.stop();
    sensorTable.getEntry("isAutoAiming").setBoolean(false);
    sensorTable.getEntry("isRobotAimed").setBoolean(false);
    autoAimSubsystem.disableLimelightSnapshot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
