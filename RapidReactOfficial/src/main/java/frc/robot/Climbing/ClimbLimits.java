package frc.robot.Climbing;

// Utility class with helper functions that determine if motion of the climber should stop so we don't inccur penalties.
// Note: call safeTo...(...), not the private member functions.
public class ClimbLimits() {
    public final double safetyRoomInInches = 1.0; // Stay this far away from any bounds that the rules state.

    // Returns whether the arms can extend further (true), or if they have reached our "safe" limits (false).
    public static boolean safeToExtend(ClimberPosition currentPosition) {
        if (reachedHoriztonalLimit(currentPosition)) {
            return false;
        }
        if (reachedVerticalLimit(currentPosition)) {
            return false;
        }
        return true;
    }

    // Returns whether the arms can tilt up further (true), or if they have reached our "safe" limits (false).
    // Note: If they can't tilt up, the climber needs to retract (may already be happening).
    public static boolean safeToTiltUp(ClimberPosition currentPosition) {
        // Tilting up is limited by the max height of the robot in the hanger.
        if (reachedVerticalLimit(currentPosition)) {
            return false;
        }
        return true;
    }

    // Returns whether the arms can tilt down further (true), or if they have reached our "safe" limits (false).
    // Note: If they can't tilt up, the climber needs to retract (may already be happening).
    public static boolean safeToTiltDown(ClimberPosition currentPosition) {
        // Tilting down is limited by G107: max extension outside frame perimeter.
        if (reachedHoriztonalLimit(currentPosition)) {
            return false;
        }
        return true;
    }

    private static boolean reachedHoriztonalLimit(ClimberPosition currentPosition) {
        // Determine horizontal component of the arm.
        // We know the angle and length of the arm (hypotanues)
        double angleInRadians = Math.toRadians(currentPosition.angleInDegrees);
        double horiztontalComponent = Math.cos(angleInRadians) * currentPosition.lengthInInches;
        // Adjust based on where the tilt location is.
        double extensionBeyondFramePerimter = horiztontalComponent - ClimberSpecs.pivotCenterLengthFromBackOfChassisInInches;
        final double safeExtensionLimit = ClimberSpecs.allowedFramePerimiterExtensionInInches - safetyRoomInInches;
        if (extensionBeyondFramePerimter >= safeExtensionLimit) {
            return true;
        } else {
            return false;
        }
    }

    private static boolean reachedVerticalLimit(ClimberPosition currentPosition) {
        double angleInRadians = Math.toRadians(currentPosition.angleInDegrees);
        double verticalComponent = Math.sin(angleInRadians) * currentPosition.lengthInInches;
        // Adjust based on where the tilt location is.
        double heightAboveGroundInInches = horiztontalComponent + ClimberSpecs.pivotCenterHeightAboveGroundInInches;
        final double safeHeightLimit = ClimberSpecs.allowedMaxHeightInInches - safetyRoomInInches;
        if (heightAboveGroundInInches >= safeHeightLimit) {
            return true;
        } else {
            return false;
        }
    }
}