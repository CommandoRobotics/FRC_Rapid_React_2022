package frc.robot.Climbing;

// Encapsulates the automated steps for climbing.
// To use this: call getDesiredPosition() for the current step, control the motors to that position, then advance() to go to the next step, and repeat.
public class ClimbSequence {
    public int currentStep; // Tracks the state-machine step number

    // The following is the order. Defining it here provides one place to change if we need to add/remove a step in the sequence.
    private final int preClimb = 0; // During the match, as we drive around
    private final int midBarReachUp = 1; // Arms up (while in hanger) to grab mid bar.
    private final int midBarRetractPastShooter = 2; // Arms retract the bar past the point of the static hooks.
    private final int midBarAlignHooks = 3; // Position so that the arm hooks and shooter hooks are now both engaged on the bar.
    private final int highBarReleaseMidBar = 4; // Move the arm so it unhooks from the Mid Bar.
    private final int highBarReachOut = 5; // Tilt and extend the arms.
    private final int highBarTouchBar = 6; // Tilt up to touch the High Bar so it is in line with the hooks (not hooked yet).
    private final int highBarGrabBar = 7; // Retract so we... 1) hook the high bar, 2) pull the static (shooter) hooks off the mid bar, and 3) move the High bar beyond the static hook tip.
    private final int highBarAlignHooks = 8; // Same as midBarAlighHooks, but for high bar.
    // The following repeeat the midbar steps, but for traversal
    private final int traversalBarReleasehighBar = 9;
    private final int traversalBarReachOut = 10;
    private final int traversalBarTouchBar = 11;
    private final int traversalBarGrabBar = 12;
    private final int traversalBarAlignHooks = 13;
    private final int traversalBarRaiseTheRoof = 14; // Celebrate. Not necessary, but makes it easier to pull the robot after the bar after the match.

    public ClimbSequence() {
        currentStep = 0;
    }

    // Moves to the next step in the sequence
    public void advance() {
        ++currentStep;
         // Count begins at zero, so totalSteps is one past the last step.
        if (currentStep >= traversalBarRaiseTheRoof) {
            currentStep = traversalBarRaiseTheRoof;
        }
    }
    
    // Moves to the previous step in the sequence
    public void goBack() {
        --currentStep;
        if (currentStep < 0) {
            currentStep = 0;
        }
    }

    // Returns the length the arm should be extended to and the angle the climber should be at for the current step in the process.
    public ClimberPosition getDesiredPosition() {
        // Note that most of these cases use "return", which exits the function, so "break" is not required in that scenario.
        switch (currentStep) {
            case preClimb:
                return stowedPosition();
            case midBarReachUp:
                return hooksAboveMidBarHeight();
            case midBarRetractPastShooter:
                return retractedPastShooter();
            case midBarAlignHooks:
                return allHooksAligned();
            case highBarReleaseMidBar:
                return armsJustAboveShooterHooks();
            case highBarReachOut:
                reachedForNextBar();
            case highBarTouchBar:
                touchingNextBar();
            case highBarGrabBar:
                return retractedPastShooter();
            case highBarAlignHooks:
                return allHooksAligned();
            case traversalBarReleasehighBar:
                return armsJustAboveShooterHooks();
            case traversalBarReachOut:
                return reachedForNextBar();
            case traversalBarTouchBar:
                return touchingNextBar();
            case traversalBarGrabBar:
                return retractedPastShooter();
            case traversalBarAlignHooks:
                return allHooksAligned();
            case traversalBarRaiseTheRoof:
                return armsJustAboveShooterHooks();
            // If advanced beyond the end, just return the final position
            default:
                return armsJustAboveShooterHooks();            
        }
    }

    // Compares the passed position and returns true if the arm is near enough to move on.
    public boolean closeEnough(ClimberPosition actualPosition) {
        ClimberPosition desiredPosition = getDesiredPosition();
        double armLengthDeltaInInches = Math.abs(desiredPosition.lengthInInches - actualPosition.lengthInInches);
        double tiltDeltaInDegrees = Math.abs(desiredPosition.angleInDegrees - actualPosition.angleInDegrees);

        double lengthToleranceInInches = 0.5; // Extended too long or too short by this much is considered OK.
        double tiltTolereanceInDegrees = 1.5; // Tilted too low or high by this many degrees is considered OK.
        
        // You can tune individual steps here, using the commented code below
        // example:
        //if (step == traversalBarRaiseTheRoof) {
        //    lengthToleranceInInches = 5.0;
        //}
        
        boolean lengthGood = false;
        if (armLengthDeltaInInches < lengthToleranceInInches) {
            lengthGood = true;
        }
        boolean angleGood = false;
        if (tiltDeltaInDegrees < tiltTolereanceInDegrees) {
            angleGood = true;
        }
        boolean allGood = false;
        if (lengthGood && angleGood) {
            allGood = true;
        }
        return allGood;
    }

    // Returns the position of the climber when stowed during teleop.
    // Note: Publicly accessible in case we want to call it during the match. Other positions should be accesed via getDesiredPosition()
    public static ClimberPosition stowedPosition() {
        return new ClimberPosition(ClimberSpecs.fullyRetractedLengthInInches, 90.0);
    }

    // Returns the position of the climber vertical, just above the mid bar, in prepreation for driving to it.
    private ClimberPosition hooksAboveMidBarHeight() {
        // TODO: Tune this value. We want our hook to fully clear the bar, but don't want it too much higher.
        final double midBarHookHeightInInches = 65.0; // Distance from ground to top edge of hook.
        final double midBarExtensionLengthInInches = midBarHookHeightInInches - pivotCenterHeightAboveGroundInInches;
        return new ClimberPosition(midBarExtensionLengthInInches, 90.0);
    }

    // Returns the position of the climber when the arm hooks are just below (and in front of) the shooter hooks
    private ClimberPosition retractedPastShooter() {
        // TODO: Tune this value. We want the retracting hooks to pull the bar below the static (shooter hooks), but not so low that it crushes the top edge of the shooter.
        final double armHookBelowShooterHookHeightInInches = 32.0; // Distance from the ground to the top edge of the hook when it is below the shooter hook.
        final double belowShooterHookExtensionLengthInInches = armHookBelowShooterHookHeightInInches - pivotCenterHeightAboveGroundInInches;
        return new ClimberPosition(belowShooterHookExtensionLengthInInches, 95.0);
    }

    // Returns the position where the arm hooks and the shooter hooks are lined up.
    private ClimberPosition allHooksAligned() {
        // TODO: Tune this value. Align the hooks perfectly.
        final double armHookatShooterHookHeightInInches = 34.0; // Distance from the ground to the top edge of the hook when it is below the shooter hook.
        final double atShooterHookExtensionLengthInInches = armHookatShooterHookHeightInInches - pivotCenterHeightAboveGroundInInches;
        return new ClimberPosition(atShooterHookExtensionLengthInInches, 90.0);        
    }

    // Returns the position where the arm hooks have moved enough above the shooter hooks so they are completely free of the bar and won't snag it as they move to the next position.
    private ClimberPosition armsJustAboveShooterHooks() {
        // TODO: Tune this value. Extend the arm so the arm hooks are above the shooter hooks and are completely away from the bar.
        final double armHookaboveShooterHookHeightInInches = 36.0; // Distance from the ground to the top edge of the hook when it is below the shooter hook.
        final double aboveShooterHookExtensionLengthInInches = armHookatShooterHookHeightInInches - pivotCenterHeightAboveGroundInInches;
        return new ClimberPosition(aboveShooterHookExtensionLengthInInches, 90.0); 
    }

    // Returns the position of the arms tilted down and extended beyond the next bar.
    private ClimberPosition reachedForNextBar() {
        // TODO: Tune these values.
        final double armTiltedBelowNextBarAngleInDegrees = 135.0; // Tilt the arm so it is below the next bar.
        final double armReachingBeyondNextBarLengthInInches = 48.0; // Extend the arm so it is beyond the next bar (when tilted up).
        return new ClimberPosition(armReachingBeyondNextBarLengthInInches, armTiltedBelowNextBarAngleInDegrees);
    }

    // Returns the position where the arms are touching the next bar, but the hooks are still just beyond it.
    private ClimberPosition touchingNextBar() {
        // TODO: Tune these values.
        final double armAtNextBarAngleInDegrees = 120.0; // Tilt the arm so it is touching the next bar.
        final double armReachingJustBeyondNextBarLengthInInches = 48.0; // Extend the arm so it is now just barely beyond the next bar (giving enough wiggle room for error).
        return new ClimberPosition(armReachingJustBeyondNextBarLengthInInches, armAtNextBarAngleInDegrees);
    }
}
