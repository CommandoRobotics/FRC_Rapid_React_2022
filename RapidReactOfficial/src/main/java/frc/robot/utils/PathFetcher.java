package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class PathFetcher {

    static double defaultMaxVel = 8;
    static double defaultMaxAccel = 5;

    static PathPlannerTrajectory[] doubleShot = new PathPlannerTrajectory[2];

    /**
     * Load all paths
     */
    public static void loadAllPaths() {
        doubleShot[0] = PathPlanner.loadPath(
            "Double Shot 1", 
            defaultMaxVel, 
            defaultMaxAccel
            );
        doubleShot[1] = PathPlanner.loadPath(
            "Double Shot 2", 
            defaultMaxVel, 
            defaultMaxAccel);
    }

    /**
     * Get all doubleShot paths
     * @return All doubleShot paths
     */
    public static PathPlannerTrajectory[] fetchDoubleShot() {
        return doubleShot;
    }

    /**
     * Get all doubleShot paths
     * @param The part of this whole path to fetch
     * @return All doubleShot paths
     */
    public static PathPlannerTrajectory fetchDoubleShot(int part) {
        return doubleShot[part];
    }

}
