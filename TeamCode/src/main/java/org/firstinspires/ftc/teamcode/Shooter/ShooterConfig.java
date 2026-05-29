package org.firstinspires.ftc.teamcode.Shooter;

/**
 * ShooterConfig — central constants for the FTC Decode flywheel shooter.
 *
 * CONTROL STRATEGY ("KSV" feedforward + P controller)
 * ---------------------------------------------------
 *   motorOutput = feedForward + pTerm
 *   feedForward = (KV * targetRPM) + KS
 *   pTerm       =  KP * (targetRPM - actualRPM)
 *
 * No integral term (windup is catastrophic on a high-inertia flywheel).
 * No derivative term (damping fights the inertia we want to preserve).
 *
 * Tune in this order, using the OpModes in the Shooter package:
 *   Step 1: KSTune.java    — static friction floor
 *   Step 2: KVTune.java    — velocity feedforward (steady-state power per RPM)
 *   Step 3: KPTune.java    — proportional gain (recovery from ball-firing dips)
 *   Step 4: VerifyTune.java — validate full system meets match-ready criteria
 *
 * Hardware: GoBILDA Yellow Jacket 5203-2402-0001 (6000 RPM free, 28 PPR at motor shaft)
 *
 * IMPORTANT — encoder reads at the MOTOR SHAFT (pre-gear-reduction).
 * If your gear reduction is 3:1, then 1500 RPM at the flywheel = 4500 RPM at
 * the encoder. Pick TARGET_RPM accordingly.
 */
public final class ShooterConfig {

    // -------------------------------------------------------------------------
    // Hardware device names — must match REV Control Hub configuration
    // -------------------------------------------------------------------------

    /**
     * Flywheel motors. The shooter uses TWO motors on opposite sides of
     * the flywheel. Both drive the same wheel, so both receive the same
     * power command every loop.
     *
     * shooter1 — original motor, mounted in the original orientation.
     * shooter2 — second motor on the OPPOSITE side. Because it is
     *            mechanically mirrored, it runs REVERSE so a positive
     *            power command spins the flywheel the same physical
     *            direction as shooter1.
     *
     * Both motors have encoder cables to the Expansion Hub, but the
     * control loop reads velocity from shooter1 only — one flywheel,
     * one speed. shooter2's encoder is available for cross-checking
     * (see ShooterSubsystem.getMotor2RPM()) but is not used for control.
     */
    public static final String FLYWHEEL_MOTOR_1_NAME = "shooter1";
    public static final String FLYWHEEL_MOTOR_2_NAME = "shooter2";

    /**
     * Name of the motor that physically feeds the shooter. On this robot
     * the rear intake doubles as the indexer (hopper -> shooter).
     *
     * NOTE: This constant is informational only. IntakeSubsystem hard-codes
     * "rearIntake" in its feedShooter() path. Kept here so tuning OpModes
     * can reference it if needed in the future.
     */
    public static final String INDEXER_MOTOR_NAME  = "rearIntake";

    // -------------------------------------------------------------------------
    // Motor specs
    // -------------------------------------------------------------------------

    /** Encoder ticks per revolution at the motor shaft (GoBILDA 5203 internal). */
    public static final double TICKS_PER_REV = 28.0;

    /** Motor free-speed RPM at 12 V, no load. */
    public static final double MOTOR_FREE_SPEED_RPM = 6000.0;

    // -------------------------------------------------------------------------
    // Target velocity
    // -------------------------------------------------------------------------

    /**
     * Default target motor-shaft RPM for tuning OpModes.
     * 3000 motor RPM = 1000 RPM at the flywheel after the 3:1 gear reduction.
     * Increase toward 4500 motor RPM (= 1500 wheel RPM) for longer shots.
     */
    public static final double TARGET_RPM = 3000.0;

    /** Acceptable steady-state velocity error (± RPM) to be considered "at speed". */
    public static final double VELOCITY_TOLERANCE_RPM = 75.0;

    /**
     * How long actual RPM must stay within tolerance before the
     * "ready to fire" flag goes true. Prevents firing during a transient
     * zero-crossing of the error signal during ramp-up.
     */
    public static final long READY_DWELL_MS = 200;

    // -------------------------------------------------------------------------
    // KSV controller coefficients — tune in order: KS, then KV, then KP
    // -------------------------------------------------------------------------

    /**
     * KS — static friction floor as a motor power fraction (0.0 - 1.0).
     * Tune with KSTune: smallest power that JUST makes the flywheel rotate.
     * Typical range: 0.02 - 0.10.
     */
    public static final double KS_INITIAL = 0.0950;

    /**
     * KV — velocity feedforward (power per unit of target RPM).
     * Tune with KVTune: with KS fixed and KP=0, raise KV until actual RPM
     * matches target at steady state.
     *
     * Rule of thumb: KV ~ (1.0 - KS) / MOTOR_FREE_SPEED_RPM
     *   (1.0 - 0.04) / 6000 ~ 0.000160  (no-load estimate)
     *   Loaded flywheels typically need ~0.00018 - 0.00021.
     *
     * NOTE: the prior value of 0.000400 was approximately 2x too high and
     * caused the controller to saturate at full output.
     */
    public static final double KV_INITIAL = 0.000175;

    /**
     * KP — proportional gain (corrective power per RPM of error).
     * Tune with KPTune AFTER KS and KV: raise until recovery from a shot
     * is fast without oscillation. Typical range: 0.0003 - 0.005.
     *
     * NOTE: the prior value of 0.010 was almost certainly too high given
     * the saturated KV. Re-tune from a small starting point.
     */
    public static final double KP_INITIAL = 0.0045;

    // -------------------------------------------------------------------------
    // Live-tuning increment sizes (used by KSTune/KVTune/KPTune)
    // -------------------------------------------------------------------------

    public static final double KS_STEP = 0.005;
    public static final double KV_STEP = 0.000005;
    public static final double KP_STEP = 0.0005;
    public static final double RPM_STEP = 50.0;

    // -------------------------------------------------------------------------
    // Velocity ramp rate
    // -------------------------------------------------------------------------

    /**
     * Maximum RPM change per second when ramping the target.
     * Set 0 to disable (instant jump). Used by tuning OpModes — match
     * code applies the target directly since the motor itself handles
     * its acceleration profile.
     */
    public static final double RAMP_RATE_RPM_PER_SEC = 3000.0;

    // -------------------------------------------------------------------------
    // Indexer / ball-feed behaviour
    // -------------------------------------------------------------------------

    public static final int BURST_COUNT = 3;
    public static final long INDEXER_ON_TIME_MS  = 1000;
    public static final long INDEXER_OFF_TIME_MS = 250;   // was 0 — caused stacked feeds
    public static final double INDEXER_POWER = 1.0;
    public static final boolean WAIT_FOR_FLYWHEEL_READY = true;

    // -------------------------------------------------------------------------
    // Single-shot fire timing (for the discrete RT-trigger fire command)
    // -------------------------------------------------------------------------

    /** How long the stopper servo stays open per discrete shot. */
    public static final long STOPPER_OPEN_DURATION_MS = 250;

    /** Minimum time between consecutive shots — gives flywheel time to recover. */
    public static final long MIN_INTER_SHOT_MS = 200;

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    public static final long TELEMETRY_UPDATE_MS = 50;

    /** Shared button debounce for all gamepad input across tuning OpModes. */
    public static final long BUTTON_DEBOUNCE_MS = 200;

    // -------------------------------------------------------------------------
    // Safety
    // -------------------------------------------------------------------------

    /**
     * If actualRPM exceeds targetRPM by this factor while commanded on,
     * the controller will force shutdown to protect the motor and gear train.
     * Used by tuning OpModes and the match shooter subsystem.
     */
    public static final double OVERSPEED_TRIP_FACTOR = 1.20;

    private ShooterConfig() {}
}
