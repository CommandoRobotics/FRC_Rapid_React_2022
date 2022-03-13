package frc.robot.Projectile;

public class Range implements Comparable<Range>{

    int id;

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
        // Convert min value to an int and set it as the ID.
        // This is necessary for this object to be comparable.
        id = Math.round((float)minValue*10);
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

    @Override
    public int compareTo(Range object) {
        return id-object.id;
    }
    
}
