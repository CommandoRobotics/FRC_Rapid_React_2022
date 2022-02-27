// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerSubsystem extends SubsystemBase {

  NetworkTableInstance nInst;
  NetworkTable powerUsage;
  PowerDistribution hub;

  final int vrmChannel = 0;
  final int driveFLChannel = 0;
  final int driveFRChannel = 0;
  final int driveRLChannel = 0;
  final int driveRRChannel = 0;
  final int intakeChannel = 0;
  final int rampChannel = 0;
  final int shooterLChannel = 0;
  final int shooterRChannel = 0;
  final int transferLChannel = 0;
  final int transferRChannel = 0;
  final int verticalChannel = 0;

  /** Creates a new PowerSubsystem. */
  public PowerSubsystem() {
    nInst = NetworkTableInstance.getDefault();
    powerUsage = nInst.getTable("CommandoDash").getSubTable("PowerUsage");
    hub = new PowerDistribution(1, ModuleType.kRev);
  }

  /**
   * Updates the power draw network tables that are used by CommandoDash.
   */
  public void updatePowerDraw() {
    // Update battery voltage
    powerUsage.getEntry("batteryVoltage").setDouble(hub.getVoltage());

    // Update total current draw
    powerUsage.getEntry("totalCurrent").setDouble(hub.getTotalCurrent());

    // Update VRM current draw
    powerUsage.getEntry("VRMCurrent").setDouble(hub.getCurrent(vrmChannel));

    // Update drive motor current draw
    powerUsage.getEntry("driveFLCurrent").setDouble(hub.getCurrent(driveFLChannel));
    powerUsage.getEntry("driveFRCurrent").setDouble(hub.getCurrent(driveFRChannel));
    powerUsage.getEntry("driveRLCurrent").setDouble(hub.getCurrent(driveRLChannel));
    powerUsage.getEntry("driveRRCurrent").setDouble(hub.getCurrent(driveRRChannel));

    // Update intake current draw
    powerUsage.getEntry("intakeCurrent").setDouble(hub.getCurrent(intakeChannel));

    // Update index current draw
    powerUsage.getEntry("transferLCurrent").setDouble(hub.getCurrent(transferLChannel));
    powerUsage.getEntry("transferRCurrent").setDouble(hub.getCurrent(transferRChannel));
    powerUsage.getEntry("rampCurrent").setDouble(hub.getCurrent(rampChannel));
    powerUsage.getEntry("verticalCurrent").setDouble(hub.getCurrent(verticalChannel));

    // Update shooter current draw
    powerUsage.getEntry("shooterLCurrent").setDouble(hub.getCurrent(shooterLChannel));
    powerUsage.getEntry("shooterRCurrent").setDouble(hub.getCurrent(shooterRChannel));
  }

  @Override
  public void periodic() {
    updatePowerDraw();
  }
}
