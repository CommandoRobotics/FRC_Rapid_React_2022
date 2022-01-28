// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsPorts;
import frc.robot.Constants.ConstantsValues;

public class DriveSubsystem extends SubsystemBase {

  // Motor controllers
  CANSparkMax frontLeftSpark, frontRightSpark, rearLeftSpark, rearRightSpark;

  // Drive encoders
  RelativeEncoder frontLeftEncoder, frontRightEncoder, rearLeftEncoder, rearRightEncoder;

  SparkMaxPIDController frontLeftPidController, frontRightPidController, rearLeftPidController, rearRightPidController;

  // NavX
  AHRS navX;

  // Other mecanum drive utilities
  MecanumDrive drive;
  MecanumDriveOdometry odometry;
  Field2d field;

  public DriveSubsystem() {

    // Instantiate the Spark Maxes
    frontLeftSpark = new CANSparkMax(ConstantsPorts.frontLeftSparkId, MotorType.kBrushless);
    frontRightSpark = new CANSparkMax(ConstantsPorts.frontRightSparkId, MotorType.kBrushless);
    rearLeftSpark = new CANSparkMax(ConstantsPorts.rearLeftSparkId, MotorType.kBrushless);
    rearRightSpark = new CANSparkMax(ConstantsPorts.rearRightSparkId, MotorType.kBrushless);

    // Invert the Spark Maxes
    frontLeftSpark.setInverted(false);
    frontRightSpark.setInverted(false);
    rearLeftSpark.setInverted(false);
    rearRightSpark.setInverted(false);

    // Instantiate the drive encoders
    frontLeftEncoder = frontLeftSpark.getEncoder();
    frontRightEncoder = frontRightSpark.getEncoder();
    rearLeftEncoder = rearLeftSpark.getEncoder();
    rearRightEncoder = rearRightSpark.getEncoder();

    // Invert the drive encoders
    frontLeftEncoder.setInverted(false);
    frontRightEncoder.setInverted(false);
    rearLeftEncoder.setInverted(false);
    rearRightEncoder.setInverted(false);

    // Set the drive encoder conversion factors
    frontLeftEncoder.setVelocityConversionFactor(ConstantsValues.distancePerMotorRotationMeters);
    frontRightEncoder.setVelocityConversionFactor(ConstantsValues.distancePerMotorRotationMeters);
    rearLeftEncoder.setVelocityConversionFactor(ConstantsValues.distancePerMotorRotationMeters);
    rearRightEncoder.setVelocityConversionFactor(ConstantsValues.distancePerMotorRotationMeters);
    //TODO add position conversion factors

    // Instantiate the drive PID controllers
    frontLeftPidController = frontLeftSpark.getPIDController();
    frontRightPidController = frontRightSpark.getPIDController();
    rearLeftPidController = rearLeftSpark.getPIDController();
    rearRightPidController = rearRightSpark.getPIDController();

    // Set PID controller values
    frontLeftPidController.setP(ConstantsValues.driveWheelP);
    frontLeftPidController.setI(ConstantsValues.driveWheelI);
    frontLeftPidController.setD(ConstantsValues.driveWheelD);
    frontLeftPidController.setIZone(ConstantsValues.driveWheelIZone);
    frontLeftPidController.setFF(ConstantsValues.driveWheelFeedForward);
    frontLeftPidController.setOutputRange(ConstantsValues.driveWheelMinOutput, ConstantsValues.driveWheelMaxOutput);
    frontRightPidController.setP(ConstantsValues.driveWheelP);
    frontRightPidController.setI(ConstantsValues.driveWheelI);
    frontRightPidController.setD(ConstantsValues.driveWheelD);
    frontRightPidController.setIZone(ConstantsValues.driveWheelIZone);
    frontRightPidController.setFF(ConstantsValues.driveWheelFeedForward);
    frontRightPidController.setOutputRange(ConstantsValues.driveWheelMinOutput, ConstantsValues.driveWheelMaxOutput);
    rearLeftPidController.setP(ConstantsValues.driveWheelP);
    rearLeftPidController.setI(ConstantsValues.driveWheelI);
    rearLeftPidController.setD(ConstantsValues.driveWheelD);
    rearLeftPidController.setIZone(ConstantsValues.driveWheelIZone);
    rearLeftPidController.setFF(ConstantsValues.driveWheelFeedForward);
    rearLeftPidController.setOutputRange(ConstantsValues.driveWheelMinOutput, ConstantsValues.driveWheelMaxOutput);
    rearRightPidController.setP(ConstantsValues.driveWheelP);
    rearRightPidController.setI(ConstantsValues.driveWheelI);
    rearRightPidController.setD(ConstantsValues.driveWheelD);
    rearRightPidController.setIZone(ConstantsValues.driveWheelIZone);
    rearRightPidController.setFF(ConstantsValues.driveWheelFeedForward);
    rearRightPidController.setOutputRange(ConstantsValues.driveWheelMinOutput, ConstantsValues.driveWheelMaxOutput);

    // Instantiate the NavX
    navX = new AHRS(SPI.Port.kMXP);

    // Instantiate other mecanum drive utilities
    drive = new MecanumDrive(frontLeftSpark, rearLeftSpark, frontRightSpark, rearRightSpark);
    field = new Field2d();

    // Add our field to the smart dash
    SmartDashboard.putData("Field", field);
  }

  /**
   * Set the speed of the front left spark
   * @param speed
   */
  public void setFrontLeftSpeed(double speed) {
    frontLeftSpark.set(speed);
  }

  /**
   * Set the speed of the front right spark
   * @param speed
   */
  public void setFrontRightSpeed(double speed) {
    frontRightSpark.set(speed);
  }

  /**
   * Set the speed of the rear left spark
   * @param speed
   */
  public void setRearLeftSpeed(double speed) {
    rearLeftSpark.set(speed);
  }

  /**
   * Set the speed of the rear right spark
   * @param speed
   */
  public void setRearRightSpeed(double speed) {
    rearRightSpark.set(speed);
  }

  /**
   * Set the volts of the front left spark
   * @param volts
   */
  public void setFrontLeftVolts(double volts) {
    frontLeftSpark.setVoltage(volts);
  }

  /**
   * Set the volts of the front left spark
   * @param volts
   */
  public void setFrontRightVolts(double volts) {
    frontRightSpark.setVoltage(volts);
  }

  /**
   * Set the volts of the front left spark
   * @param volts
   */
  public void setRearLeftVolts(double volts) {
    rearLeftSpark.setVoltage(volts);
  }

  /**
   * Set the volts of the front left spark
   * @param volts
   */
  public void setRearRightVolts(double volts) {
    rearRightSpark.setVoltage(volts);
  }

  /**
   * Set all motors to the same speed, resulting in the robot moving forward
   * @param speed
   */
  public void driveForward(double speed) {
    setFrontLeftSpeed(speed);
    setFrontRightSpeed(speed);
    setRearLeftSpeed(speed);
    setRearRightSpeed(speed);
  }

  /**
   * Set all motors to the same voltage, resulting in the robot moving forward
   * @param volts
   */
  public void driveForwardVolts(double volts) {
    setFrontLeftVolts(volts);
    setFrontRightVolts(volts);
    setRearLeftVolts(volts);
    setRearRightVolts(volts);
  }

  /**
   * Stop the drivetrain
   */
  public void stop() {
    driveForward(0);
  }

  /**
   * Get the front left encoder position
   * @return The front left encoder position
   */
  public double getFrontLeftPosition() {
    return frontLeftEncoder.getPosition();
  }

  /**
   * Get the front right encoder position
   * @return THe front right encoder position
   */
  public double getFrontRightPosition() {
    return frontRightEncoder.getPosition();
  }

  /**
   * Get the rear left encoder position
   * @return The rear left encoder position
   */
  public double getRearLeftPosition() {
    return rearLeftEncoder.getPosition();
  }

  /**
   * Get the rear right encoder position
   * @return The rear right encoder position
   */
  public double getRearRightPosition() {
    return rearRightEncoder.getPosition();
  }

  /**
   * Get the current wheel speeds
   * @return A MecanumDriveWheelSpeeds object containing the current wheel speeds in meters per second
   */
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      frontLeftEncoder.getVelocity(), 
      frontRightEncoder.getVelocity(), 
      rearLeftEncoder.getVelocity(), 
      rearRightEncoder.getVelocity());
  }

  /**
   * Set the wheel voltages
   * @param wheelVoltages A MecanumDriveMotorVoltages object containing the voltages to set each wheel to
   */
  public void setVoltages(MecanumDriveMotorVoltages wheelVoltages) {
    frontLeftSpark.setVoltage(wheelVoltages.frontLeftVoltage);
    frontRightSpark.setVoltage(wheelVoltages.frontRightVoltage);
    rearLeftSpark.setVoltage(wheelVoltages.rearLeftVoltage);
    rearRightSpark.setVoltage(wheelVoltages.rearRightVoltage);
  }

  /**
   * Set the drive wheel speeds
   * @param wheelSpeeds A MecanumDriveWheelSpeeds object containing the speeds to set each wheel to
   */
  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    frontLeftPidController.setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
    frontRightPidController.setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
    rearLeftPidController.setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
    rearRightPidController.setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
  }

  /**
   * Get the heading of the robot in degrees
   * @return The heading of the robot in degrees
   */
  public double getHeading() {
    return navX.getAngle();
  }

  /**
   * Reset all drive encoders
   */
  public void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    rearLeftEncoder.setPosition(0);
    rearRightEncoder.setPosition(0);
  }

  /**
   * Reset the gyro
   */
  public void resetGyro() {
    navX.reset();
  }

  /**
   * Drive the robot using a y, x, and rotation value
   * @param y
   * @param x
   * @param rotation
   */
  public void driveMecanum(DoubleSupplier y, DoubleSupplier x, DoubleSupplier rotation) {
    drive.driveCartesian(y.getAsDouble(), x.getAsDouble(), rotation.getAsDouble());
  }

  /**
   * Drive the robot either field centrically or not field centrically with a y, x, and rotation value
   * @param y
   * @param x
   * @param rotation
   * @param fieldCentric True if the robot should drive field centrically, false if it should not.
   */
  public void driveMecanum(DoubleSupplier y, DoubleSupplier x, DoubleSupplier rotation, boolean fieldCentric) {
    if(fieldCentric) {
      drive.driveCartesian(y.getAsDouble(), x.getAsDouble(), rotation.getAsDouble(), getHeading());
    } else {
      drive.driveCartesian(y.getAsDouble(), x.getAsDouble(), rotation.getAsDouble());
    }
  }

  /**
   * Get the robot pose in meters
   * @return A Pose2D object representing the robot pose in meters
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), getWheelSpeeds());
    field.setRobotPose(odometry.getPoseMeters());
  }
}