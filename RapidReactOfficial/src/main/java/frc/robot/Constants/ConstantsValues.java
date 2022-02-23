package frc.robot.Constants;

import java.util.TreeMap;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Projectile.Range;
import frc.robot.Projectile.Vector;

public class ConstantsValues {
    
    /**
     * DRIVE
     */
    public static final double driveDeadband = 0.1;
    // PID and feedforward values for each wheel on the drivetrain
    public static final double driveWheelP = 0;
    public static final double driveWheelI = 0;
    public static final double driveWheelD = 0;
    public static final double driveWheelIZone = 0;
    public static final double driveWheelFeedForward = 0;
    public static final double driveWheelMinOutput = 0;
    public static final double driveWheelMaxOutput = 0;

    // PID values for drivetrain y
    public static final double driveYP = 0;
    public static final double driveYI = 0;
    public static final double driveYD = 0;

    // PID values for drivetrain x
    public static final double driveXP = 0;
    public static final double driveXI = 0;
    public static final double driveXD = 0;

    // PID values for drivetrain rotation
    public static final double driveRotationP = 0;
    public static final double driveRotationI = 0;
    public static final double driveRotationD = 0;

    // Encoder conversion factor math and finals
    public static final double wheelDiameterMeters = Units.inchesToMeters(6);
    public static final double wheelCircumferenceMeters = wheelDiameterMeters*Math.PI;
    public static final double driveGearRatio = 8.45;
    public static final double distancePerMotorRotationMeters = wheelCircumferenceMeters/driveGearRatio;
    
    // Motor translations relative to the center of the robot
    public static final double trackWidthMeters = 0; // Distance between the left and right wheels
    public static final double wheelBaseMeters = 0; // Distance between the very bottom of the front and rear wheels
    public static final MecanumDriveKinematics mecanumDriveKinematics = new MecanumDriveKinematics(
        new Translation2d(wheelBaseMeters/2, trackWidthMeters/2), 
        new Translation2d(wheelBaseMeters/2, -trackWidthMeters/2), 
        new Translation2d(-wheelBaseMeters/2, trackWidthMeters/2), 
        new Translation2d(-wheelBaseMeters/2, -trackWidthMeters/2)
        );

    // Misc drive values
    public static double maxWheelVelocityMetersPerSecond;
    public static final double rotationMaxVelocityMetersPerSec = 0;
    public static final double rotationMaxAccelerationMetersPerSecPerSec = 0;
    public static final double mecanumFeedForwardKS = 0;
    public static final double mecanumFeedForwardKV = 0;
    public static final SimpleMotorFeedforward mecanumFeedForward = new SimpleMotorFeedforward(mecanumFeedForwardKS, mecanumFeedForwardKV);

    /**
     * SHOOTER
     */
    //TODO tune shooter PID values
    public static final double flywheelP = 0;
    public static final double flywheelI = 0;
    public static final double flywheelD = 0;
    public static final double flywheelIZone = 0;
    public static final double flywheelFF = 0;
    public static final double flywheelMinOutput = -1;
    public static final double flywheelMaxOutput = 1;

    // Shooter feedforward values
    public static final double flywheelKs = 0;
    public static final double flywheelKv = 0;
    public static final double flywheelKa = 0;

    // Vector tree map
    public static final TreeMap<Range, Vector> vectorMap = new TreeMap<Range, Vector>();

    // Other shooter values
    public static final double shooterHeightMeters = Units.inchesToMeters(10);
    public static final double defaultKickwheelRpm = 0;
    public static final double defaultKickwheelVoltage = 0;

    // Limelight values
    public static final double limelightMountingAngle = 17; // The angle at which the Limelight is mounted above the horizon in degrees.

    public static final double flywheelExpelVolts = 0.2;

    /*
    Methods for adding values to vector treemap
    */
    /**
    * Add an entry with a given range and vector object to the VectorMap
    * @param range The range object
    * @param vector The vector objects
    */
    public static void addToVectorMap(Range range, Vector vector) {
        ConstantsValues.vectorMap.put(range, vector);
    }

    /**
    * Add an entry with given values to the VectorMap
    * @param minRange The minimum range value this vector applies to
    * @param maxRange The maximum range value this vector applies to
    * @param velocity The velocity of the vector
    * @param angle The angle of the vector
    */
    public static void addToVectorMap(double minRange, double maxRange, double velocity, double angle) {
        ConstantsValues.vectorMap.put(new Range(minRange, maxRange), new Vector(velocity, angle));
    }

    /**
     * INTAKE
     */
    public static final double intakePower = 0;
    public static final double ejectPower = 0;

    /**
     * AUTO AIM
     */
    // PID
    public static final double panP = 0;
    public static final double panI = 0;
    public static final double panD = 0;
    public static final double panSetPoint = 0;
    public static final double tiltP = 0;
    public static final double tiltI = 0;
    public static final double tiltD = 0;
    public static final double tiltSetPoint = 0;

    public static final double panPositionConversionFactor = 1;
    public static final double tiltPositionConversionFactor = 1;

    public static final double panPidDeadzone = 0.05;
    public static final double panPidMaxOutput = 0.5;

    public static final double manualPanDeadband = 0.05;

    /**
     * INDEX
     */
    public static final double rampPositionConversionFactor = 1;
    public static final double verticalPositionConversionFactor = 1;
    public static final double rampVelocityConversionFactor = 1;
    public static final double verticalVelocityConversionFactor = 1;
    public static final double shooterBlockTime = 500;
    public static final double indexSensorThreshold = 1000;

    public static final double transferIntakeSpeed = 0.5;
    public static final double rampIntakeSpeed = 0.3;
    public static final double verticalExpelSpeed = 0.3;
    public static final double rampExpelSpeed = -0.3;
    public static final double transferExpelSpeed = -0.5;

    public static final double verticalP = 0;
    public static final double verticalI = 0;
    public static final double verticalD = 0;
    public static final double verticalIZone = 0;
    public static final double verticalFF = 0;
    public static final double verticalMinOutput = -1;
    public static final double verticalMaxOutput = -1;

    public static final double defaultVerticalVelocity = 0;

    public static final double rampJogSpeed = 0.3;
    public static final double verticalJogSpeed = 0.3;
    public static final double transferJogSpeed = 0.5;




}
