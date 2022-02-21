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
    public static final double topFlywheelP = 0;
    public static final double topFlywheelI = 0;
    public static final double topFlywheelD = 0;
    public static final double topFlywheelIZone = 0;
    public static final double topFlywheelFF = 0;
    public static final double topFlywheelMinOutput = -1;
    public static final double topFlywheelMaxOutput = 1;

    public static final double bottomFlywheelP = 0;
    public static final double bottomFlywheelI = 0;
    public static final double bottomFlywheelD = 0;
    public static final double bottomFlywheelIZone = 0;
    public static final double bottomFlywheelFF = 0;
    public static final double bottomFlywheelMinOutput = -1;
    public static final double bottomFlywheelMaxOutput = 1;

    public static final double kickwheelP = 0;
    public static final double kickwheelI = 0;
    public static final double kickwheelD = 0;
    public static final double kickwheelIZone = 0;
    public static final double kickwheelFF = 0;
    public static final double kickwheelMinOutput = -1;
    public static final double kickwheelMaxOutput = 1;

    // Shooter feedforward values
    public static final double topFlywheelKs = 0;
    public static final double topFlywheelKv = 0;
    public static final double topFlywheelKa = 0;

    public static final double bottomFlywheelKs = 0;
    public static final double bottomFlywheelKv = 0;
    public static final double bottomFlywheelKa = 0;
    
    public static final double kickwheelKs = 0;
    public static final double kickwheelKv = 0;
    public static final double kickwheelKa = 0;

    // Vector tree map
    public static final TreeMap<Range, Vector> vectorMap = new TreeMap<Range, Vector>();

    // Other shooter values
    public static final double shooterHeightMeters = Units.inchesToMeters(10);
    public static final double defaultKickwheelRpm = 0;
    public static final double defaultKickwheelVoltage = 0;

    // Limelight values
    public static final double limelightMountingAngle = 17; // The angle at which the Limelight is mounted above the horizon in degrees.


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

    //CargoHound PID values
    public static final double houndXP = 0.5;
    public static final double houndXI = 0;
    public static final double houndXD = 0;

    public static final double houndYP = 0.5;
    public static final double houndYI = 0;
    public static final double houndYD = 0;

    public static final double houndMaxRVel = 10;
    public static final double houndMaxRAcc = 5;
    public static final double houndRP = 0.5;
    public static final double houndRI = 0;
    public static final double houndRD = 0;
    public static final TrapezoidProfile.Constraints houndRConstraints =
        new TrapezoidProfile.Constraints(houndMaxRVel, houndMaxRAcc);

    public static final double noCargoTime = 0.5;
    public static final double minHoundPIDOut = 0.1;
}
