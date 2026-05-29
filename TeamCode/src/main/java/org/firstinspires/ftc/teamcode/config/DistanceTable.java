package org.firstinspires.ftc.teamcode.config;

/**
 * DistanceTable holds the distance-to-power lookup grid and the
 * interpolation logic used by both teleop and autonomous.
 *
 * Distances are in inches (Limelight reports inches after our conversion).
 * Power values are motor power fractions (0.0 - 1.0) which the shooter
 * subsystem converts to RPM via {@code power * MOTOR_FREE_SPEED_RPM}.
 *
 * Power values themselves live in AllianceConfig because Blue and Red
 * needed slightly different presets in practice. Distances are shared.
 */
public final class DistanceTable {

    /** Distance grid in inches, monotonically increasing. */
    public static final double[] DISTANCE_PRESETS =
            {24.0, 36.0, 48.0, 60.0, 72.0, 84.0, 120.0, 132.0};

    /** Display names matching DISTANCE_PRESETS, for telemetry. */
    public static final String[] DISTANCE_NAMES =
            {"2 ft", "3 ft", "4 ft", "5 ft", "6 ft", "7 ft", "10 ft", "11 ft"};

    /**
     * Linear interpolation between the two nearest distance presets.
     * Clamps to the table endpoints if the distance falls outside the range.
     *
     * @param distanceInches measured distance from Limelight
     * @param powerPresets   the alliance-specific power array; must be the
     *                       same length as DISTANCE_PRESETS
     * @return motor power fraction (0.0 - 1.0)
     */
    public static double interpolatePower(double distanceInches, double[] powerPresets) {
        if (powerPresets.length != DISTANCE_PRESETS.length) {
            throw new IllegalArgumentException(
                    "powerPresets length " + powerPresets.length +
                            " does not match DISTANCE_PRESETS length " + DISTANCE_PRESETS.length);
        }

        // Clamp below
        if (distanceInches <= DISTANCE_PRESETS[0]) {
            return powerPresets[0];
        }
        // Clamp above
        if (distanceInches >= DISTANCE_PRESETS[DISTANCE_PRESETS.length - 1]) {
            return powerPresets[powerPresets.length - 1];
        }

        // Find the bracketing pair and lerp
        for (int i = 0; i < DISTANCE_PRESETS.length - 1; i++) {
            if (distanceInches >= DISTANCE_PRESETS[i] && distanceInches <= DISTANCE_PRESETS[i + 1]) {
                double t = (distanceInches - DISTANCE_PRESETS[i])
                        / (DISTANCE_PRESETS[i + 1] - DISTANCE_PRESETS[i]);
                return powerPresets[i] + t * (powerPresets[i + 1] - powerPresets[i]);
            }
        }

        // Should be unreachable given the clamps above, but be defensive
        return powerPresets[0];
    }

    private DistanceTable() {}
}
