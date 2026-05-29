package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

/**
 * Alliance identifies which side of the field the robot is playing on.
 *
 * Used to select per-alliance constants (AprilTag IDs, Limelight pipelines,
 * shooter power offsets) and to mirror autonomous paths from the canonical
 * Blue-side definition.
 *
 * MIRROR CONVENTION
 * -----------------
 * The FTC field is 144 inches wide. Blue starts on one side (low X) and
 * Red on the other (high X). To mirror a Blue pose to Red:
 *   X'       = 144 - X
 *   Y'       = Y           (unchanged — same axis)
 *   heading' = PI - heading  (reflect about the X=72 vertical axis)
 *
 * All autonomous routes are defined in BLUE coordinates exactly once.
 * Red OpModes get their poses by calling {@link #mirror(Pose)} on the
 * Blue definitions at runtime. This eliminates the transcription bugs
 * we had when Blue/Red were maintained as separate files.
 */
public enum Alliance {
    BLUE,
    RED;

    /** Field width in inches — mirror axis is at FIELD_WIDTH / 2. */
    public static final double FIELD_WIDTH = 144.0;

    /**
     * Returns the supplied Blue-side pose unchanged for Blue alliance,
     * or the mirrored equivalent for Red alliance.
     *
     * Always pass in the BLUE pose. The method handles per-alliance
     * conversion internally.
     */
    public Pose mirror(Pose bluePose) {
        if (this == BLUE) return bluePose;
        return new Pose(
                FIELD_WIDTH - bluePose.getX(),
                bluePose.getY(),
                Math.PI - bluePose.getHeading()
        );
    }

    /**
     * Mirrors a heading (radians) only, for use with
     * setLinearHeadingInterpolation calls.
     */
    public double mirrorHeading(double blueHeadingRadians) {
        if (this == BLUE) return blueHeadingRadians;
        return Math.PI - blueHeadingRadians;
    }
}
