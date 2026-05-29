package org.firstinspires.ftc.teamcode.subsystems;

/**
 * DriveUtils contains the joystick-conditioning math shared by the teleop
 * base class and any subsystem that takes raw stick input.
 *
 * Improvements over the original square deadzone:
 *   - Radial deadzone (uses magnitude, not per-axis abs) — eliminates the
 *     diagonal drift that happened when both axes were just past the
 *     square deadzone but the joint magnitude was tiny.
 *   - Smooth post-deadzone scaling — output is 0 at the deadzone edge and
 *     ramps to 1 at full deflection, preserving precision near zero.
 *   - Optional squared-with-sign curve for finer slow control.
 */
public final class DriveUtils {

    public static final double DEFAULT_DEADBAND = 0.05;

    /** Single-axis deadzone with smooth post-deadband scaling. */
    public static double applyDeadzone(double value, double deadband) {
        double abs = Math.abs(value);
        if (abs < deadband) return 0;
        return Math.signum(value) * (abs - deadband) / (1.0 - deadband);
    }

    /** Single-axis deadzone with default deadband. */
    public static double applyDeadzone(double value) {
        return applyDeadzone(value, DEFAULT_DEADBAND);
    }

    /**
     * Radial deadzone + smooth scaling for a 2D stick.
     * Returns a 2-element array {x, y}.
     */
    public static double[] applyRadialDeadzone(double x, double y, double deadband) {
        double magnitude = Math.hypot(x, y);
        if (magnitude < deadband) return new double[]{0, 0};
        double scale = (magnitude - deadband) / (1.0 - deadband) / magnitude;
        return new double[]{x * scale, y * scale};
    }

    /** Squared-with-sign curve: preserves sign, gives more fine control near zero. */
    public static double squareCurve(double value) {
        return value * Math.abs(value);
    }

    private DriveUtils() {}
}
