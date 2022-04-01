package frc.robot.Climbing;

// Encapsulates a climber position (arm length and angle)
public class ClimberPosition {

    public double lengthInInches;
    public double angleInDegrees;

        // Calibrate the values below by tilting the climber to and from its extremes.
        final private double lowestAngleDegrees = 35.0; // TODO: Calibrate this value.
        final private double lowestAngleEncoderTicks = 1250.0; // TODO: Calibrate this value.
        final private double highestAngleDegrees = 100.0; // TODO: Calibrate this value.
        final private double highestAngleEncoderTicks = 0.0; // TODO: Calibrate this value.
        // Calibrate the values below by fully retracting and extending the climber.
        final double retractedLengthInches = 32.0; // TODO: Calibrate this value.
        final double retractedPositionTicks = 0.0; // TODO: Calibrate this value.
        final double extendedLengthInches = 70.0; // TODO: Calibrate this value.
        final double extendedPositionTicks = 10000.0; // TODO: Calibrate this value.     

    /**
     * Constructor
     * @param lengthInInches Distance from center of pivot point to outside edge of hook.
     * @param angleInDegrees Angle based on 0° = horizontal toward intake and straight up is 90°.
     */
    public Climber(double desiredLengthInInches, double desiredAngleInDegrees) {
        lengthInInches = desiredLengthInInches;
        angleInDegrees = desiredAngleInDegrees;
    }

    // Constructs a Climber instance based on encoder ticks
    public static Climber fromEncoderValues(double winchTicks, double tiltTicks) {
        double armLengthInInches = lengthInInchesfromEncoderTicks(winchTicks);
        double tiltAngleInDegrees = angleInDegreesfromEncoderTicks(tiltTicks);
        return new Climber(armLengthInInches, tiltAngleInDegrees);
    }

    /**
     * Provides the encoder value of the tilting motor to achieve this ClimberPosition angle.
     * Note: If the angle is outside the calibrated range, it will return the appropriate min/max.
     * @return Number of encoder "ticks". Should be able to feed this to the motor controller.
     */
    public double tiltEncoderTicks() {
        convertUnits(angleInDegrees, lowestAngleDegrees, lowestAngleEncoderTicks, highestAngleDegrees, highestAngleEncoderTicks);
    }

    /**
     * Provides the encoder value of the winch motor to achieve this ClimberPosition length.
     * Note: If the length is outside the calibrated range, it will return the appropriate min/max.
     * @return Number of encoder "ticks". Should be able to feed this to the motor controller.
     */
    public double winchEncoderTicks() {
        return convertUnits(lengthInInches, retractedLengthInches, retractedPositionTicks, extendedLengthInches, extendedPositionTicks);
    }

    // Returns the angle of the climber arms, given the passed encoder value.
    // Note: If the encoder value is outside the calibrated values, the closest calibrated angle will be returned.
    public double angleInDegreesfromEncoderTicks(double ticks) {
        return convertUnits(ticks, lowestAngleEncoderTicks, lowestAngleDegrees, highestAngleEncoderTicks, highestAngleDegrees);
    }
    
    // Returns the length of the climber arms, given the passed encoder value.
    // Note: If the encoder value is outside the calibrated values, the closest calibrated angle will be returned.
    public double lengthInInchesfromEncoderTicks(double ticks) {
        return convertUnits(ticks, retractedPositionTicks, retractedLengthInches, extendedPositionTicks, extendedLengthInches);
    }

    /**
    * Converts from one unit system (i.e. Degrees) to the corresponding other unit system (i.e. encoder ticks).
    * Note: This handles issues where the ranges move in opposite directions (i.e. ticks increase as angle decreases).
    * @param value Value in current units.
    * @param aCalibratedPositionCurrentUnits One limit position (i.e. minimum) using the same units as the passed value.
    * @param aCalibratedPositionDesiredUnits Same limit position above, but expressed in the other units.
    * @param bCalibratedPositionCurrentUnits Other limit position (i.e. maximum) using the same units as the passed value.
    * @param bCalibratedPositionDesiredUnits Other limit position, but expressed in other units.
    * @return Value in the other units, or the closest limit position (if the value is beyond the calibrated positions).
    */
    private double convertUnits(value, aCalibratedPositionCurrentUnits, aCalibratedPositionDesiredUnits, bCalibratedPositionCurrentUnits, bCalibratedPositionDesiredUnits) {
        // Determine which calibrated position is larger, then check bounds appropriately
        if (aCalibratedPositionCurrentUnits > bCalibratedPositionCurrentUnits) {
            if (value >= aCalibratedPositionCurrentUnits) {
                return aCalibratedPositionDesiredUnits;
            }
            if (value <= bCalibratedPositionCurrentUnits) {
                return bCalibratedPositionDesiredUnits;
            }
        } else {
            if (value >= bCalibratedPositionCurrentUnits) {
                return bCalibratedPositionDesiredUnits;
            }
            if (value <= aCalibratedPositionCurrentUnits) {
                return aCalibratedPositionDesiredUnits;
            }            
        }
        
        // Determine where along the current unit number line.
        double minCurrentUnits = Math.min(aCalibratedPositionCurrentUnits, bCalibratedPositionCurrentUnits);
        double maxCurrentUnits = Math.max(aCalibratedPositionCurrentUnits, bCalibratedPositionCurrentUnits);
        double normalizedValue = value - minCurrentUnits;
        double percentageValue = normalizedValue / (maxCurrentUnits - minCurrentUnits);

        // Convert to other units
        double minDesiredUnits = Math.min(aCalibratedPositionDesiredUnits, bCalibratedPositionDesiredUnits);
        double maxDesiredUnits = Math.max(aCalibratedPositionDesiredUnits, bCalibratedPositionDesiredUnits);
        double normalizedInDesiredUnits = percentageValue * (maxDesiredUnits - minDesiredUnits);
        double valueInDesiredUnits = normalizedInDesiredUnits + minDesiredUnits;
        return valueInDesiredUnits;
    }
}
