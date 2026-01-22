package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LL_Turret_Config {
    // Limelight
    public static int LL_PIPELINE = 5;                 // AprilTag pipeline index
    public static boolean APPLY_PIPELINE_LIVE = false; // only turn on if you truly need to change pipeline mid-run

    // Turret mapping
    public static double TURRET_CENTER = 0.50;
    public static double TURRET_MIN = 0.00;
    public static double TURRET_MAX = 1.00;

    // Servo travel model:
    // If full servo travel corresponds to ~90 degrees of turret sweep, keep 90.0.
    public static double TURRET_RANGE_DEG = 90.0;

    // Aiming behavior
    public static double AIM_TOLERANCE_DEG = 2.0;

    // Calibration offsets / direction
    public static double TX_ZERO_OFFSET_DEG = 0.0;     // subtract this from tx so "on target" becomes ~0
    public static boolean TURRET_INVERT = false;       // flip if your turret moves the wrong way

    // Optional: rate limiting (helps prevent servo jitter)
    public static double MAX_SERVO_STEP_PER_LOOP = 0.03; // 0 disables if you set very large like 1.0
}
