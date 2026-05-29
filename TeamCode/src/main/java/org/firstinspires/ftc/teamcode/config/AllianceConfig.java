package org.firstinspires.ftc.teamcode.config;

/**
 * AllianceConfig groups together every value that differs between
 * Blue and Red alliance into one immutable object per side.
 *
 * Previously these were duplicated across BlueTeleopv2 / RedTeleopv2 /
 * BlueAuto* / RedAuto* — eight or more places where a value could drift.
 * Now there are exactly two instances: {@link #BLUE} and {@link #RED}.
 *
 * To add a new per-alliance value: add a final field, add it to the
 * constructor, and update the two static instances. Done.
 */
public final class AllianceConfig {

    /** AprilTag ID for this alliance's scoring target (goal). */
    public final int aprilTagId;

    /** Limelight pipeline index pre-configured for this alliance's tag. */
    public final int limelightPipeline;

    /**
     * Power level used by the B-button auto-move feature in teleop.
     * Picked at the AUTO_MOVE_TARGET_DISTANCE pose; slightly different
     * between alliances because the goal geometry differs subtly.
     */
    public final double autoMoveShooterPower;

    /**
     * Per-alliance power preset lookup table.
     * Indices align with {@link DistanceTable#DISTANCE_PRESETS}.
     */
    public final double[] powerPresets;

    private AllianceConfig(int aprilTagId,
                           int limelightPipeline,
                           double autoMoveShooterPower,
                           double[] powerPresets) {
        this.aprilTagId           = aprilTagId;
        this.limelightPipeline    = limelightPipeline;
        this.autoMoveShooterPower = autoMoveShooterPower;
        this.powerPresets         = powerPresets;
    }

    // -------------------------------------------------------------------------
    // The two alliance configurations. Edit these and only these.
    // -------------------------------------------------------------------------

    public static final AllianceConfig BLUE = new AllianceConfig(
            /* aprilTagId            */ 20,
            /* limelightPipeline     */ 1,
            /* autoMoveShooterPower  */ 0.35,
            /* powerPresets          */ new double[]{0.30, 0.33, 0.35, 0.37, 0.39, 0.415, 0.475, 0.50}
    );

    public static final AllianceConfig RED = new AllianceConfig(
            /* aprilTagId            */ 24,
            /* limelightPipeline     */ 2,
            /* autoMoveShooterPower  */ 0.35,
            /* powerPresets          */ new double[]{0.30, 0.33, 0.35, 0.37, 0.39, 0.415, 0.475, 0.50}
    );

    /** Look up the AllianceConfig that corresponds to a given Alliance enum. */
    public static AllianceConfig forAlliance(Alliance alliance) {
        return alliance == Alliance.BLUE ? BLUE : RED;
    }
}
