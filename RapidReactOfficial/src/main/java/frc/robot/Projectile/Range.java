package frc.robot.Projectile;

public class Range {

    public double minValue;
    public double maxValue;

    /**
     * Create a new range with a min and a max value
     * @param minValue The min value
     * @param maxValue The max value
     */
    public Range(double minValue, double maxValue) {
        this.minValue = minValue;
        this.maxValue = maxValue;
    }

    /**
     * Checks if the given value is within the range contained by this object
     * Note: This checks if the given value is greater than or equal to the minimum
     * value and less than the maximum value. This prevents interference between values.
     * @param valueToCheck The value of which to check 
     * @return A boolean representing if the given value is represented by the range of this object.
     */
    public boolean isValueInRange(double valueToCheck) {
        return valueToCheck >= minValue && valueToCheck < maxValue;
    }
    
}
