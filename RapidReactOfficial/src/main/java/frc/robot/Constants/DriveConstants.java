package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    
    // Motor controller ports/IDs
    public static final int frontLeftSparkId= 0;
    public static final int frontRightSparkId = 0;
    public static final int rearLeftSparkId = 0;
    public static final int rearRightSparkId = 0;

    // Encoder conversion factor math and finals
    public static final double wheelDiameterMeters = Units.inchesToMeters(6);
    public static final double wheelCircumferenceMeters = wheelDiameterMeters*Math.PI;
    public static final double driveGearRatio = 8.45;
    public static final double distancePerMotorRotationMeters = wheelCircumferenceMeters/driveGearRatio;
    

    // Motor translations relative to the center of the robot
    public static final double trackWidthMeters = 0; // Distance between the left and right wheels
    public static final double wheelBaseMeters = 0; // Distance between the very bottom of the front and rear wheels
    public static final MecanumDriveKinematics driveKinematics = new MecanumDriveKinematics(
        new Translation2d(wheelBaseMeters/2, trackWidthMeters/2), 
        new Translation2d(wheelBaseMeters/2, -trackWidthMeters/2), 
        new Translation2d(-wheelBaseMeters/2, trackWidthMeters/2), 
        new Translation2d(-wheelBaseMeters/2, -trackWidthMeters/2)
        );

}
