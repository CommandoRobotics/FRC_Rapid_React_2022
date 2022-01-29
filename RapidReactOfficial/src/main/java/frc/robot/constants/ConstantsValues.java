package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ConstantsValues {

    // Shooter PID values
    //TODO tune shooter PID values
    public static final double flywheelP = 0;
    public static final double flywheelI = 0;
    public static final double flywheelD = 0;
    public static final double flywheelIZone = 0;
    public static final double flywheelFeedForward = 0;
    public static final double flywheelMinOutput = 0;
    public static final double flywheelMaxOutput = 0;

    public static final double kickwheelP = 0;
    public static final double kickwheelI = 0;
    public static final double kickwheelD = 0;
    public static final double kickwheelIZone = 0;
    public static final double kickwheelFeedForward = 0;
    public static final double kickwheelMinOutput = 0;
    public static final double kickwheelMaxOutput = 0;

    // Other shooter values
    public static final double shooterHeightMeters = Units.inchesToMeters(10);

    // Limelight values
    public static final double limelightMountingAngle = 17; // The angle at which the Limelight is mounted above the horizon in degrees.

}
