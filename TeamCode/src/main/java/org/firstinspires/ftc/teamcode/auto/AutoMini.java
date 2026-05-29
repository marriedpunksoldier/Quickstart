package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Alliance;

/**
 * AutoMini — start on far side, drive to one shoot pose, fire 3, sweep.
 *
 * Identical structure to AutoQuick but with a different starting position
 * for matches where the robot lines up on the far side of the field.
 */
public abstract class AutoMini extends AutoBase {

    protected AutoMini(Alliance alliance) {
        super(alliance);
    }

    private static final Pose START     = new Pose(24.448105436573318, 130.41186161449755, Math.toRadians(145));
    private static final Pose SHOOT1    = new Pose(57.89785831960461, 104.79736408566721, Math.toRadians(143));
    private static final Pose WAYPOINT1 = new Pose(19.63920922570016, 101.21746293245471, Math.toRadians(-90));

    private static final double EXPECTED_DISTANCE_SHOOT1 = 55.0;

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
                    intake.stop();
                    follower.followPath(pathToWaypoint1, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Done
                break;
        }
    }
}
