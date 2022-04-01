package frc.robot.Climbing;

// Encapsulates constants used by multiple climber-related classes, or that may change mechanically (and don't want to have to dig for them in code to change them).
public class ClimberSpecs {
    // Robot Properties
    // TODO: Meaasure and set all these values
    public static final double pivotCenterHeightAboveGroundInInches = 1.11111; // Distance from ground up to the center of the hex shaft that the climber arms pivot on.
    public static final double pivotCenterLengthFromFrontOfChassisInInches = 1.11111; // Distance from the front (shooter side) edge of the chassis (frame perimeter, not bumpers).
    public static final double chassisLengthInInches = 1.11111; // Length of the chassis, front to back (edges of frame perimeter, not bumpers).
    public static final double fullyRetractedHeightInInches = 1.11111; // Height above the GROUND of the hooks (top edge) when fully retracted.
    public static final double winchGearboxRatio = 9; // Multiply all stages of gearbox to get this number
    public static final double tiltGearboxRatio = 15; // Multiply all stages of gearbox to get this number

    // Rules
    public static final double allowedFramePerimiterExtensionInInches = 16.0; // Distance we are allowed to extend in any direction from the frame perimter.
    public static final double allowedMaxHeightInInches = 5 * 12 + 6; // Max height in Hangar Zone is 5'6".

    // Derived
    public static final double pivotCenterLengthFromBackOfChassisInInches = chassisLengthInInches - pivotCenterLengthFromFrontOfChassisInInches; // Distance from the front (shooter side) edge of the chassis (frame perimeter, not bumpers).
    public static final double fullyRetractedLengthInInches = fullyRetractedHeightInInches - pivotCenterHeightAboveGroundInInches; // Length of the climber arm (from center of pivot to edge of hook) when fully retracted)
}
