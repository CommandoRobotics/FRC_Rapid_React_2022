package frc.robot.Constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public class ConstantsValues {
    
    // PID and feedforward values for each wheel on the drivetrain
    public static final double driveWheelP = 0;
    public static final double driveWheelI = 0;
    public static final double driveWheelD = 0;
    public static final double driveWheelIZone = 0;
    public static final double driveWheelFeedForward = 0;
    public static final double driveWheelMinOutput = 0;
    public static final double driveWheelMaxOutput = 0;

    // PID and Feedforward values for y, x, and rotation of the drivetrain
    public static final double driveYP = 0;
    public static final double driveYI = 0;
    public static final double driveYD = 0;

    public static final double driveXP = 0;
    public static final double driveXI = 0;
    public static final double driveXD = 0;

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

    public static double maxWheelVelocityMetersPerSecond;
    public static final double rotationMaxVelocityMetersPerSec = 0;
    public static final double rotationMaxAccelerationMetersPerSecPerSec = 0;
    public static final double mecanumFeedForwardKS = 0;
    public static final double mecanumFeedForwardKV = 0;
    public static final SimpleMotorFeedforward mecanumFeedForward = new SimpleMotorFeedforward(mecanumFeedForwardKS, mecanumFeedForwardKV);

}
