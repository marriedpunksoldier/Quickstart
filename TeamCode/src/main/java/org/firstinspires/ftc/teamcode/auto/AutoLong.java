package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Alliance;

/**
 * AutoLong — three shooting positions with two sample-sweep cycles.
 * Replaces BlueAutoLongv3 and RedAutoLongv3.
 *
 * Total possible: 9 balls scored over ~25 seconds.
 *
 * Route:
 *   START -> SHOOT1 (3 balls) -> sweep WP1-WP2-WP3 -> SHOOT2 (3 balls)
 *         -> sweep WP4-WP5-WP6 -> SHOOT3 (3 balls) -> park at WP7
 */
public abstract class AutoLong extends AutoBase {

    protected AutoLong(Alliance alliance) {
        super(alliance);
    }

    // Blue-side poses (Red mirrors automatically)
    private static final Pose START   = new Pose(56, 8, Math.toRadians(90));
    private static final Pose SHOOT1  = new Pose(59.54530477759472, 16.843492586490928, Math.toRadians(115));
    private static final Pose SHOOT2  = new Pose(59.54530477759472, 16.843492586490928, Math.toRadians(115));
    private static final Pose SHOOT3  = new Pose(59.54530477759472, 16.843492586490928, Math.toRadians(115));

    private static final Pose WP1 = new Pose(53.14, 35.1104, Math.toRadians(180));
    private static final Pose WP2 = new Pose(24.197693574958812, 35.82207578253706, Math.toRadians(180));
    private static final Pose WP3 = new Pose(53.14, 35.1104, Math.toRadians(180));
    private static final Pose WP4 = new Pose(52.6656, 60.0198, Math.toRadians(180));
    private static final Pose WP5 = new Pose(24.197693574958805, 59.782537067545285, Math.toRadians(180));
    private static final Pose WP6 = new Pose(52.6656, 60.0198, Math.toRadians(180));
    private static final Pose WP7 = new Pose(38.02800658978585, 13.26688632619441, Math.toRadians(90));

    // Expected distances at each shoot pose (MEASURE AT FIELD).
    private static final double EXPECTED_DIST_SHOOT1 = 50.0;
    private static final double EXPECTED_DIST_SHOOT2 = 50.0;
    private static final double EXPECTED_DIST_SHOOT3 = 50.0;

    private PathChain p1_toShoot1;
    private PathChain p2_toWp1;
    private PathChain p3_toWp2;
    private PathChain p4_toWp3;
    private PathChain p5_toShoot2;
    private PathChain p6_toWp4;
    private PathChain p7_toWp5;
    private PathChain p8_toWp6;
    private PathChain p9_toShoot3;
    private PathChain p10_toWp7;

    @Override
    protected Pose getStartPose() {
        return START;
    }

    @Override
    protected void buildPath() {
        Pose start = alliance().mirror(START);
        Pose s1    = alliance().mirror(SHOOT1);
        Pose s2    = alliance().mirror(SHOOT2);
        Pose s3    = alliance().mirror(SHOOT3);
        Pose w1    = alliance().mirror(WP1);
        Pose w2    = alliance().mirror(WP2);
        Pose w3    = alliance().mirror(WP3);
        Pose w4    = alliance().mirror(WP4);
        Pose w5    = alliance().mirror(WP5);
        Pose w6    = alliance().mirror(WP6);
        Pose w7    = alliance().mirror(WP7);

        p1_toShoot1  = straightPath(start, s1);
        p2_toWp1     = straightPath(s1, w1);
        p3_toWp2     = straightPath(w1, w2);
        p4_toWp3     = straightPath(w2, w3);
        p5_toShoot2  = straightPath(w3, s2);
        p6_toWp4     = straightPath(s2, w4);
        p7_toWp5     = straightPath(w4, w5);
        p8_toWp6     = straightPath(w5, w6);
        p9_toShoot3  = straightPath(w6, s3);
        p10_toWp7    = straightPath(s3, w7);
    }

    @Override
    protected void onPathUpdate() {
        switch (pathState()) {
            // === Position 1 ===
            case 0:
                preSpinForDistance(EXPECTED_DIST_SHOOT1);
                follower.followPath(p1_toShoot1, true);
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
                    follower.followPath(p2_toWp1, true);
                    setPathState(3);
                }
                break;

            // === Sweep 1: WP1 -> WP2 -> WP3 ===
            case 3:
                if (atDestination()) {
                    beginSweep(p3_toWp2);
                    setPathState(4);
                }
                break;

            case 4:
                if (atDestination()) {
                    endSweep();
                    follower.followPath(p4_toWp3, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (atDestination()) {
                    preSpinForDistance(EXPECTED_DIST_SHOOT2);
                    follower.followPath(p5_toShoot2, true);
                    setPathState(6);
                }
                break;

            // === Position 2 ===
            case 6:
                if (atDestination()) {
                    refineRPMFromLimelight();
                    beginShootBurst(2);
                    setPathState(7);
                }
                break;

            case 7:
                if (updateShootBurst()) {
                    shooter.stop();
                    follower.followPath(p6_toWp4, true);
                    setPathState(8);
                }
                break;

            // === Sweep 2: WP4 -> WP5 -> WP6 ===
            case 8:
                if (atDestination()) {
                    beginSweep(p7_toWp5);
                    setPathState(9);
                }
                break;

            case 9:
                if (atDestination()) {
                    endSweep();
                    follower.followPath(p8_toWp6, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (atDestination()) {
                    preSpinForDistance(EXPECTED_DIST_SHOOT3);
                    follower.followPath(p9_toShoot3, true);
                    setPathState(11);
                }
                break;

            // === Position 3 ===
            case 11:
                if (atDestination()) {
                    refineRPMFromLimelight();
                    beginShootBurst(3);
                    setPathState(12);
                }
                break;

            case 12:
                if (updateShootBurst()) {
                    shooter.stop();
                    follower.followPath(p10_toWp7, true);
                    setPathState(13);
                }
                break;

            case 13:
                // Done — auto-stow kicks in at t=29s
                break;
        }
    }
}
