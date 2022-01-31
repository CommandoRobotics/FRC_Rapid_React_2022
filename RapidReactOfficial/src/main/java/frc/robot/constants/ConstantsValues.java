package frc.robot.constants;

import edu.wpi.first.math.util.Units;

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


    // Other shooter values
    public static final double shooterHeightMeters = Units.inchesToMeters(10);

    // Limelight values
    public static final double limelightMountingAngle = 17; // The angle at which the Limelight is mounted above the horizon in degrees.

}
