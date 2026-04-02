package org.firstinspires.ftc.teamcode.Shooter;

/**
 * ShooterConfig.java
 *
 * Central constants file for the FTC 2025-2026 Decode flywheel shooter tuning suite.
 * All tunable parameters live here. Never redefine these values inline in OpModes.
 *
 * =========================================================================
 * CONTROL STRATEGY — "PVS" Controller (as taught in the tuning video)
 * =========================================================================
 * This system does NOT use standard PIDF.  The video instructor explicitly says:
 *   - NO integral  (kI) — integral windup is catastrophic on a high-inertia flywheel
 *   - NO derivative (kD) — damping force fights the flywheel's rotational inertia
 *
 * The full motor output equation is:
 *
 *   motorOutput = feedForward + pTerm
 *
 * where:
 *   feedForward = (KV * targetRPM) + KS
 *   pTerm       =  KP * (targetRPM - actualRPM)
 *
 * In plain English:
 *   KS  — gets us moving  (overcomes static friction with a constant voltage floor)
 *   KV  — keeps us at speed  (scales power linearly with target RPM)
 *   KP  — adapts to disturbances  (corrects the drop when a ball fires through)
 *
 * Tuning order (one coefficient at a time, in this exact sequence):
 *   Step 1 — Tune KS  →  run KSTune.java
 *   Step 2 — Tune KV  →  run KVTune.java (was FeedforwardTune)
 *   Step 3 — Tune KP  →  run KPTune.java (was PIDTune)
 *   Step 4 — Validate →  run VerifyTune.java
 *
 * =========================================================================
 * GEAR RATIO DESIGN (from video — 75% efficiency sweet spot rule)
 * =========================================================================
 *   Launch goal RPM (ball exit speed at 2 m distance):  1500 RPM
 *   Ideal motor shaft RPM = goalRPM / 0.75           :  2000 RPM
 *   Motor free speed (GoBILDA 5203-2402-0001)         :  6000 RPM
 *   Required gear reduction = 6000 / 2000             :  ~3:1
 *
 *   Running at 75% of free speed places the motor at its peak power+efficiency
 *   region on the torque curve, giving maximum torque headroom to recover after
 *   a ball is fired through the flywheel.
 *
 * =========================================================================
 * Hardware: GoBILDA Yellow Jacket 5203-2402-0001 (6000 RPM nominal)
 * Flywheel: DcMotorEx on REV Control Hub
 * Indexer:  DcMotor  on REV Expansion Hub (non-encoded, open-loop)
 * =========================================================================
 */
public final class ShooterConfig {

    // -------------------------------------------------------------------------
    // Hardware device names — must match your REV Control Hub configuration file
    // -------------------------------------------------------------------------

    /** DcMotorEx name for the flywheel motor on the REV Control Hub. */
    public static final String FLYWHEEL_MOTOR_NAME = "shooter";

    /** DcMotor name for the indexer (ball-feed) motor on the REV Expansion Hub. */
    public static final String INDEXER_MOTOR_NAME  = "frontIntake";

    // -------------------------------------------------------------------------
    // Motor specs
    // -------------------------------------------------------------------------

    /**
     * Encoder ticks per revolution at the MOTOR output shaft (before any external gearing).
     * GoBILDA 5203-2402-0001 internal encoder = 28 PPR at the motor shaft.
     * The video reads encoder at the motor shaft (one of two motors),
     * so do not multiply by the external gear ratio here.
     */
    public static final double TICKS_PER_REV = 28.0;

    /** Motor free-speed RPM at 12 V with no load. */
    public static final double MOTOR_FREE_SPEED_RPM = 6000.0;

    // -------------------------------------------------------------------------
    // Target velocity (directly from video)
    // -------------------------------------------------------------------------

    /**
     * Target launch RPM — stated as 1500 RPM throughout the video for a
     * 2-meter shooting distance.  This is the RPM at the flywheel output
     * (after the ~3:1 gear reduction from motor to wheel).
     *
     * Adjust in 50 RPM increments during field trials.
     * To convert from launch goal to motor shaft RPM:
     *   motorShaftRPM = launchGoalRPM / gearReduction  (e.g. 1500 / (1/3) = 4500? No—
     *   the reduction slows the wheel: motorRPM = flywheelRPM * gearRatio = 1500 * 3 = 4500)
     *   But for our encoder, we read the motor shaft.  Set TARGET_RPM to whatever
     *   shaft RPM corresponds to your desired flywheel exit speed.
     *   Video example: flywheel goal 1500 RPM at a 3:1 reduction → motor shaft reads
     *   ~4500 RPM, but the video controls to 1500 at the output directly.
     *   Match your setup: if your encoder is on the flywheel shaft, use 1500.
     *   If your encoder is on the motor shaft (pre-reduction), use 4500.
     */
    public static final double TARGET_RPM = 1500.0;

    /**
     * Acceptable steady-state velocity error (± RPM).
     * Flywheel is considered "at speed" when |actual − target| ≤ this value.
     */
    public static final double VELOCITY_TOLERANCE_RPM = 75.0;

    /**
     * How long (ms) velocity must stay within tolerance before the interlock
     * releases and allows the indexer to fire.
     */
    public static final long READY_DWELL_MS = 200;

    // -------------------------------------------------------------------------
    // PVS Controller coefficients
    // Tune in order: KS first, then KV, then KP.
    // -------------------------------------------------------------------------

    /**
     * KS — Static feedforward (power floor to overcome static friction).
     *
     * The video: "Our controller assumes zero volts = zero RPM, but in reality
     * you might have to send 0.5 V just to get the flywheel off the ground."
     * KS is that voltage floor, expressed as a motor power fraction (0.0–1.0).
     *
     * Tune with KSTune.java:
     *   Slowly raise KS until the flywheel JUST begins rotating, then back off
     *   one step.  Typical value: 0.02 – 0.10.
     */
    public static final double KS_INITIAL = 0.0300;

    /**
     * KV — Velocity feedforward (steady-state power per unit of target RPM).
     *
     * The video: "KV keeps us at the right speed."
     * feedForward = KV * targetRPM + KS
     *
     * Tune with KVTune.java:
     *   With KS fixed and KP = 0, increase KV until actual RPM matches target
     *   at steady state (no load).  Goal: |error| < 50 RPM with KP doing nothing.
     *
     * Rough estimate: KV ≈ (1.0 − KS) / MOTOR_FREE_SPEED_RPM
     *   e.g. (1.0 − 0.05) / 6000 ≈ 0.000158   (if reading motor shaft RPM)
     *   or  (1.0 − 0.05) / 1500 ≈ 0.000633   (if reading flywheel output RPM)
     */
    public static final double KV_INITIAL = 0.000183;

    /**
     * KP — Proportional gain (correction power per unit of velocity error).
     *
     * The video: "KP allows us to adapt to drops in the system" (ball firing).
     * pTerm = KP * (targetRPM − actualRPM)
     *
     * Tune with KPTune.java:
     *   With KS and KV fixed, increase KP until the flywheel recovers quickly
     *   after a ball fires — without oscillating through the target.
     *   Too high → RPM oscillates.  Too low → slow recovery after each shot.
     *
     * Typical range: 0.001 – 0.010
     */
    public static final double KP_INITIAL = 0.0025;

    // -------------------------------------------------------------------------
    // Live-tuning increment sizes
    // -------------------------------------------------------------------------

    /** KS change per button press in KSTune. */
    public static final double KS_STEP = 0.005;

    /** KV change per button press in KVTune. */
    public static final double KV_STEP = 0.00005;

    /** KP change per button press in KPTune. */
    public static final double KP_STEP = 0.0005;

    /** Target RPM change per trigger press in any tuning OpMode. */
    public static final double RPM_STEP = 50.0;

    // -------------------------------------------------------------------------
    // Velocity ramp rate
    // -------------------------------------------------------------------------

    /**
     * Maximum RPM change per second when ramping up to target.
     * The video's 0.24-second spin-up goal requires a fast ramp — keep this
     * high once KS and KV are properly tuned.  Set 0 to disable (instant jump).
     */
    public static final double RAMP_RATE_RPM_PER_SEC = 3000.0;

    // -------------------------------------------------------------------------
    // Indexer / ball-feed behavior
    // -------------------------------------------------------------------------

    /** Number of balls fired per burst-fire button press. */
    public static final int BURST_COUNT = 3;

    /**
     * Time the indexer motor runs per ball in burst mode (ms).
     * Increase if a ball is not fully clearing the indexer on each cycle.
     */
    public static final long INDEXER_ON_TIME_MS = 150;

    /**
     * Pause between balls in burst mode (ms).
     * The video's KP tuning goal is fast RPM recovery — once KP is tuned,
     * this inter-ball gap can be reduced.
     */
    public static final long INDEXER_OFF_TIME_MS = 200;

    /** Open-loop motor power for the indexer (0.0 – 1.0). */
    public static final double INDEXER_POWER = 0.85;

    /**
     * Interlock: when true, indexer will not fire until flywheel is within
     * VELOCITY_TOLERANCE_RPM of TARGET_RPM for at least READY_DWELL_MS.
     * The video implies waiting for spin-up before feeding.
     */
    public static final boolean WAIT_FOR_FLYWHEEL_READY = true;

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    /** Minimum ms between telemetry updates (avoid flooding the Driver Station). */
    public static final long TELEMETRY_UPDATE_MS = 50;

    // Private constructor — constants namespace, not instantiable.
    private ShooterConfig() {}
}
