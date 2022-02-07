package frc.robot.constants;

import java.util.TreeMap;

import edu.wpi.first.math.util.Units;
import frc.robot.Projectile.Range;
import frc.robot.Projectile.Vector;

public class ConstantsValues {

    // Shooter PID and feedforward values
    //TODO tune shooter PID values
    public static final double flywheelP = 0;
    public static final double flywheelI = 0;
    public static final double flywheelD = 0;
    public static final double flywheelIZone = 0;
    public static final double flywheelFF = 0;
    public static final double flywheelMinOutput = -1;
    public static final double flywheelMaxOutput = 1;

    public static final double kickwheelP = 0;
    public static final double kickwheelI = 0;
    public static final double kickwheelD = 0;
    public static final double kickwheelIZone = 0;
    public static final double kickwheelFF = 0;
    public static final double kickwheelMinOutput = -1;
    public static final double kickwheelMaxOutput = 1;

    // Shooter feedforward values
    public static final double flywheelKs = 0;
    public static final double flywheelKv = 0;
    public static final double flywheelKa = 0;
    
    public static final double kickwheelKs = 0;
    public static final double kickwheelKv = 0;
    public static final double kickwheelKa = 0;

    // Vector tree map
    public static final TreeMap<Range, Vector> vectorMap = new TreeMap<Range, Vector>();

    // Other shooter values
    public static final double shooterHeightMeters = Units.inchesToMeters(10);

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
}
