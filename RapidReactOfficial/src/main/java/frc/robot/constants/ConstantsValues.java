package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ConstantsValues {

    // Shooter velocity conversion factors
    //TODO set the shooter velocity conversion factors
    public static final double topFlywheelVelocityConversionFactor = 1;
    public static final double bottomFlywheelVelocityConversionFactor = 1;
    public static final double kickwheelVelocityConversionFactor = 1;

    // Shooter PID values
    //TODO set shooter PID values
    public static final double topFlywheelP = 0;
    public static final double topFlywheelI = 0;
    public static final double topFlywheelD = 0;
    public static final double topFlywheelIZone = 0;
    public static final double topFlywheelFeedForward = 0;
    public static final double topFlywheelMinOutput = 0;
    public static final double topFlywheelMaxOutput = 0;

    public static final double bottomFlywheelP = 0;
    public static final double bottomFlywheelI = 0;
    public static final double bottomFlywheelD = 0;
    public static final double bottomFlywheelIZone = 0;
    public static final double bottomFlywheelFeedForward = 0;
    public static final double bottomFlywheelMinOutput = 0;
    public static final double bottomFlywheelMaxOutput = 0;

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
