package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Alliance;

/**
 * AutoShort — two shooting positions with one sample-sweep cycle between.
 * Total: 6 balls scored if all goes well.
 */
public abstract class AutoShort extends AutoBase {

    protected AutoShort(Alliance alliance) {
        super(alliance);
    }

    // Blue-side poses
    private static final Pose START      = new Pose(24.448105436573318, 130.41186161449755, Math.toRadians(145));
    private static final Pose SHOOT1     = new Pose(57.89785831960461, 104.79736408566721, Math.toRadians(143));
    private static final Pose SHOOT2     = new Pose(57.89785831960461, 104.79736408566721, Math.toRadians(143));
    private static final Pose WAYPOINT1  = new Pose(43.9670510708402, 83.58813838550248, Math.toRadians(180));
    private static final Pose WAYPOINT2  = new Pose(17.237232289950576, 83.95551894563428, Math.toRadians(180));
    private static final Pose WAYPOINT3  = new Pose(19.63920922570016, 101.21746293245471, Math.toRadians(-90));

    private static final double EXPECTED_DISTANCE_SHOOT1 = 55.0;
    private static final double EXPECTED_DISTANCE_SHOOT2 = 55.0;

    private PathChain pathToShoot1;
    private PathChain pathToWaypoint1;
    private PathChain pathToWaypoint2;
    private PathChain pathToShoot2;
    private PathChain pathToWaypoint3;

    @Override
    protected Pose getStartPose() {
        return START;
    }

    @Override
    protected void buildPath() {
        Pose start = alliance().mirror(START);
        Pose s1    = alliance().mirror(SHOOT1);
        Pose s2    = alliance().mirror(SHOOT2);
        Pose w1    = alliance().mirror(WAYPOINT1);
        Pose w2    = alliance().mirror(WAYPOINT2);
        Pose w3    = alliance().mirror(WAYPOINT3);

        pathToShoot1    = straightPath(start, s1);
        pathToWaypoint1 = straightPath(s1, w1);
        pathToWaypoint2 = straightPath(w1, w2);
        pathToShoot2    = straightPath(w2, s2);
        pathToWaypoint3 = straightPath(s2, w3);
    }

    @Override
    protected void onPathUpdate() {
        switch (pathState()) {
            case 0:
                preSpinForDistance(EXPECTED_DISTANCE_SHOOT1);
                follower.followPath(pathToShoot1, true);
                setPathState(1);
                break;

            case 1:
                if (atDestination()) {
                    refineRPMFromLimelight();
                    beginShootBurst(1);
                    setPathState(2);
                }
                break;

            case 2:
                if (updateShootBurst()) {
                    shooter.stop();
                    follower.followPath(pathToWaypoint1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (atDestination()) {
                    beginSweep(pathToWaypoint2);
                    setPathState(4);
                }
                break;

            case 4:
                if (atDestination()) {
                    endSweep();
                    // Pre-spin for shoot 2 while driving to it
                    preSpinForDistance(EXPECTED_DISTANCE_SHOOT2);
                    follower.followPath(pathToShoot2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (atDestination()) {
                    refineRPMFromLimelight();
                    beginShootBurst(2);
                    setPathState(6);
                }
                break;

            case 6:
                if (updateShootBurst()) {
                    shooter.stop();
                    follower.followPath(pathToWaypoint3, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Done
                break;
        }
    }
}
