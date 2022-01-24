package frc.robot.projectile;

public class ProjectileMath {

    /**
     * Calculate the initial velocity - in meters per second - at which the ball must be launched in order to hit the target at a given angle.
     * @param horizontalDistanceToTarget The horizontal distance between the robot and the target in meters.
     * @param shotAngle The angle at which the ball is being shot in degrees.
     * @return The velocity at which to shoot the ball in meters per second.
     */
    public double calculateInitialVelocity(double horizontalDistanceToTarget, double shotAngle) {
        return (Math.sqrt(((horizontalDistanceToTarget*horizontalDistanceToTarget)*getAccelerationY())/(2*(getChangeInY()-horizontalDistanceToTarget*Math.toDegrees(Math.tan(shotAngle)))/Math.toDegrees(Math.cos(shotAngle)))));
    }

    /**
     * Calculate the air resistance acting on a projectile, using an inputted velocity and the drag coefficient specified in ProjectileConstants.
     * @param velocity The current velocity of the projectile in meters per second.
     * @return The force of drag acting on the ball in Newtons.
     */
    public double calculateAirResistance(double velocity) {
        return ProjectileConstants.kDragCoefficient * Math.sqrt(velocity);
    }

    /**
     * Solve a quadratic equation for the largest solution. If no real solution is found, this function will return 0 and print an error to the console.
     * @param a <b>A</b>x^2+Bx+C
     * @param b Ax^2+<b>B</b>x+C
     * @param c Ax^2+Bx+<b>C</b>
     * @return The largest solution found for the given quadratic equation.
     */
    public double solveQuadraticEquation(double a, double b, double c) {
        double discriminant = Math.sqrt(Math.pow(b,2) - 4*a*c);
        if (discriminant > 0) {
            double solA = (-b + discriminant)/(2*a);
            double solB = (-b - discriminant)/(2*a);
            if (solA > solB) {
                return solA;
            } else {
                return solB;
            }
        } if (discriminant == 0) {
            return -b/(2*a);
        } else {
            System.out.println("solveQuadraticEquation ran into impossible/imaginary numbers.");
            return 0;
        }
    }
    
    /**
     * Get the calculated acceleration in the y direction
     * @return The calculated acceleration in the y direction in meters per second squared.
     */
    public double getAccelerationY() {
        //TODO add air resistance
        return ProjectileConstants.gravity;
    }

    /**
     * Get the calculated acceleration in the x direction
     * @return The calculated acceleration in the x direction in meters per second squared.
     */
    public double getAccelerationX() {
        //TODO add air resistance
        return 0;
    }

    /**
     * Get the change in height between the robot's current height and the height of the target.
     * @return The change in height (or y) in meters.
     */
    public double getChangeInY() {
        return ProjectileConstants.kGoalHeightMeters-ProjectileConstants.kRobotHeightInMeters;
    }

}
