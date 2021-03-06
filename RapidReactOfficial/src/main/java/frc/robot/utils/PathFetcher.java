package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathFetcher {

    static double defaultMaxVel = 8;
    static double defaultMaxAccel = 3;

    static PathPlannerTrajectory[] doubleShot = new PathPlannerTrajectory[2];
    static PathPlannerTrajectory[] ideal = new PathPlannerTrajectory[4];
    static PathPlannerTrajectory[] tuning = new PathPlannerTrajectory[3];
    static PathPlannerTrajectory[] taxi = new PathPlannerTrajectory[1];
    static PathPlannerTrajectory[] difficultTestPath = new PathPlannerTrajectory[1];

    /**
     * Load all paths
     */
    public static void loadAllPaths() {
        // Load paths for ideal autonomous
        ideal[0] = PathPlanner.loadPath(
            "Ideal 1", 
            defaultMaxVel, 
            defaultMaxAccel
        );
        ideal[1] = PathPlanner.loadPath(
            "Ideal 2", 
            defaultMaxVel, 
            defaultMaxAccel
        );
        ideal[2] = PathPlanner.loadPath(
            "Ideal 3", 
            1.75, 
            defaultMaxAccel
        );
        ideal[3] = PathPlanner.loadPath(
            "Ideal 4", 
            defaultMaxVel, 
            defaultMaxAccel
        );

        // Load paths for double shot autonomous
        doubleShot[0] = PathPlanner.loadPath(
            "Double Shot 1", 
            defaultMaxVel, 
            defaultMaxAccel
        );
        taxi[0] = PathPlanner.loadPath(
            "Taxi Only", 
            defaultMaxVel, 
            defaultMaxAccel
        );
        difficultTestPath[0] = PathPlanner.loadPath(
            "Difficult Test Path", 
            defaultMaxVel, 
            defaultMaxAccel
        );
    }

    /**
     * Get a specific part of the doubleShot autonomous path
     * @param The part of this whole path to fetch
     * @return All doubleShot paths
     */
    public static PathPlannerTrajectory fetchDoubleShot(int part) {
        return doubleShot[part];
    }

    /**
     * Get a specific part of the ideal autonomous path
     * @param part The part of the ideal autonomous path specified
     * @return
     */
    public static PathPlannerTrajectory fetchIdeal(int part) {
        return ideal[part];
    }

    /**
     * Get a specific part of the taxi autonomous path
     * @param part The part of the taxi path specified
     * @return
     */
    public static PathPlannerTrajectory fetchTaxi(int part) {
        return taxi[part];
    }

    /**
     * Get a specific part of the difficult test path autonomous
     * @param part THe part of the difficult test path specified
     * @return
     */
    public static PathPlannerTrajectory fetchDifficultTestPath(int part) {
        return difficultTestPath[part];
    }
}
