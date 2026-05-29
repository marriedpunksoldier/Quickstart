package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Alliance;

/**
 * AutoQuick — drives to one shoot pose, fires 3 balls, parks at waypoint.
 * Used for short matches or as a safe fallback.
 */
public abstract class AutoQuick extends AutoBase {

    protected AutoQuick(Alliance alliance) {
        super(alliance);
    }

    // Blue-side poses (Red gets auto-mirrored)
    private static final Pose START     = new Pose(56, 8, Math.toRadians(90));
    private static final Pose SHOOT1    = new Pose(59.54530477759472, 16.843492586490928, Math.toRadians(115));
    private static final Pose WAYPOINT1 = new Pose(38.02800658978585, 13.26688632619441, Math.toRadians(-90));

    // Expected distance from SHOOT1 pose to the goal — used for pre-spin.
    // MEASURE THIS AT THE FIELD; this is a starting estimate.
    private static final double EXPECTED_DISTANCE_SHOOT1 = 50.0;

    private PathChain pathToShoot1;
    private PathChain pathToWaypoint1;

    @Override
    protected Pose getStartPose() {
        return START;
    }

    @Override
    protected void buildPath() {
        Pose start  = alliance().mirror(START);
        Pose shoot1 = alliance().mirror(SHOOT1);
        Pose park   = alliance().mirror(WAYPOINT1);

        pathToShoot1    = straightPath(start, shoot1);
        pathToWaypoint1 = straightPath(shoot1, park);
    }

    @Override
    protected void onPathUpdate() {
        switch (pathState()) {
            case 0:
                // Start the shooter spinning up while we drive
                preSpinForDistance(EXPECTED_DISTANCE_SHOOT1);
                follower.followPath(pathToShoot1, true);
                setPathState(1);
                break;

            case 1:
                if (atDestination()) {
                    // Refine RPM now that Limelight should see the tag
                    refineRPMFromLimelight();
                    beginShootBurst(1);
                    setPathState(2);
                }
                break;

            case 2:
                if (updateShootBurst()) {
                    shooter.stop();
                    intake.stop();
                    follower.followPath(pathToWaypoint1, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Done — robot will stow naturally
                break;
        }
    }
}
