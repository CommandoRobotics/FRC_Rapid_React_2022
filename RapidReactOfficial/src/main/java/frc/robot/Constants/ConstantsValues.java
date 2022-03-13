package frc.robot.Constants;

import java.util.TreeMap;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Projectile.Range;
import frc.robot.Projectile.Vector;

public class ConstantsValues {
    
    /**
     * DRIVE
     */


    // Drive limits
    public static final double driveDeadband = 0.1;
    public static final int driveCurrentLimit = 70;

    // PID and feedforward values for each wheel on the drivetrain
    public static final double driveWheelP = 3.0853;
    public static final double driveWheelI = 0;
    public static final double driveWheelD = 0;
    public static final double driveWheelIZone = 0;
    public static final double driveWheelFeedForward = 0;
    public static final double driveWheelMinOutput = 0;
    public static final double driveWheelMaxOutput = 0;

    // PID values for drivetrain y
    public static final double driveYP = 1.75;
    public static final double driveYI = 0;
    public static final double driveYD = 0;

    // PID values for drivetrain x
    public static final double driveXP = 1.75;
    public static final double driveXI = 0;
    public static final double driveXD = 0;

    // PID values for drivetrain rotation
    public static final double driveRotationP = 2.5;
    public static final double driveRotationI = 0;
    public static final double driveRotationD = 0;

    // Encoder conversion factor math and finals
    public static final double wheelDiameterMeters = Units.inchesToMeters(6);
    public static final double wheelCircumferenceMeters = wheelDiameterMeters*Math.PI;
    public static final double driveGearRatio = 8.45;
    public static final double distancePerMotorRotationMeters = ((wheelCircumferenceMeters/driveGearRatio)/42)/1.386;
    
    // Motor translations relative to the center of the robot
    public static final double trackWidthMeters = Units.inchesToMeters(22.5); // Distance between the left and right wheels
    public static final double wheelBaseMeters = Units.inchesToMeters(21); // Distance between the very bottom of the front and rear wheels
    public static final MecanumDriveKinematics mecanumDriveKinematics = new MecanumDriveKinematics(
        new Translation2d(wheelBaseMeters/2, trackWidthMeters/2), 
        new Translation2d(wheelBaseMeters/2, -trackWidthMeters/2), 
        new Translation2d(-wheelBaseMeters/2, trackWidthMeters/2), 
        new Translation2d(-wheelBaseMeters/2, -trackWidthMeters/2)
        );

    // Misc drive values
    public static double maxWheelVelocityMetersPerSecond = 10;
    public static final double rotationMaxVelocityMetersPerSec = 8;
    public static final double rotationMaxAccelerationMetersPerSecPerSec = 6;
    public static final double mecanumFeedForwardKS = 0.18366;
    public static final double mecanumFeedForwardKV = 2.1665;
    public static final SimpleMotorFeedforward mecanumFeedForward = new SimpleMotorFeedforward(mecanumFeedForwardKS, mecanumFeedForwardKV);

    /**
     * SHOOTER
     */
    //TODO tune shooter PID values
    public static double flywheelP = 0.000280;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelIZone = 0;
    public static double flywheelFF = 0;
    public static double flywheelMinOutput = -1;
    public static double flywheelMaxOutput = 1;

    // Shooter feedforward values
    public static double flywheelKs = 0.0015;
    public static double flywheelKv = 0.0021565;
    public static double flywheelKa = 0.0081225;

    public static final double shooterVelocityConversionFactor = 1;

    // Vector tree map
    public static final TreeMap<Range, Vector> vectorMap = new TreeMap<Range, Vector>();

    // Other shooter values

    public static final double flywheelVelocityConversionFactor = 1;

    //TODO adjust Limelight mounting values
    public static final double flywheelAtVelocityDeadband = 60; // Amount to be added or subtracted from the shooter velocity to determine if we're at velocity
    public static final double flywheelAtVelocityIterations = 12;

    // Limelight values
    public static double limelightMountingAngle = 35; // The angle at which the Limelight is mounted above the horizon in degrees.
    public static double shooterHeightMeters = Units.inchesToMeters(29.75);


    public static final double flywheelExpelVolts = 2;

    public static final double flywheelSecondsToSpinUp = 2000; // The number of seconds it should take to spin up the flywheel from 0 to 1.

    public static final int flywheelCurrentLimit = 40;

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
    public static final double intakePower = 0.5;
    public static final double ejectPower = -0.5;
    public static final int intakeCurrentLimit = 30;

    /**
     * AUTO AIM
     */
    // PID
    public static final double panP = 0.02;
    public static final double panI = 0;
    public static final double panD = 0;
    public static final double panSetPoint = 0;
    public static final double tiltP = 0;
    public static final double tiltI = 0;
    public static final double tiltD = 0;
    public static final double tiltSetPoint = 0;

    public static final double panPositionConversionFactor = 1;
    public static final double tiltPositionConversionFactor = 1;

    public static final double panPidDeadzone = 0.03;
    public static final double panPidMaxOutput = 0.5;
    public static final double panPidMinOutput = 0.15;
    public static final int panPidMaxIterations = 10;

    public static final double manualPanDeadband = 0.05;

    public static final double limelightPanOffset = -1;

    /**
     * INDEX
     */
    public static final double rampPositionConversionFactor = 1;
    public static final double verticalPositionConversionFactor = 1;
    public static final double rampVelocityConversionFactor = 1;
    public static final double verticalVelocityConversionFactor = 1;

    public static final double transferIntakeVolts = 3.6;
    public static final double rampIntakeVolts = 3.6;
    public static final double verticalExpelVolts = 3.6;
    public static final double rampExpelVolts = 3.6;
    public static final double transferExpelVolts = -3.6;

    public static final double rampJogVolts = 3.6;
    public static final double verticalJogVolts = 3.6;
    public static final double transferJogVolts = 3.6;

    public static final double verticalShootVolts = 4.0;


    //Camera translation relative to robot
    //TODO figure out CargoHound translation relative to robot
    private static final double houndXOffsetMeters = Units.inchesToMeters(0);
    private static final double houndYOffsetMeters = Units.inchesToMeters(0);
    public static final Translation2d houndToRobotTranslation2d = 
        new Translation2d(houndXOffsetMeters, houndYOffsetMeters);

    //CargoHound PID values
    public static final double houndXP = 0.09;
    public static final double houndXI = 0;
    public static final double houndXD = 0;

    public static final double houndYP = 0.01;
    public static final double houndYI = 0;
    public static final double houndYD = 0;

    public static final double houndMaxRVel = 3;
    public static final double houndMaxRAcc = 0.5;
    public static final double houndRP = 0.01;
    public static final double houndRI = 0;
    public static final double houndRD = 0;
    public static final TrapezoidProfile.Constraints houndRConstraints =
        new TrapezoidProfile.Constraints(houndMaxRVel, houndMaxRAcc);

    public static final double noCargoTime = 0.5;
    public static final double minHoundPIDOut = 0.15;
}
