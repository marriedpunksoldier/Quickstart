package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TurretSubsystem controls the aiming servo with two behaviours the original
 * teleop/auto files were missing:
 *
 *   1. Deadband — ignore tiny Limelight TX jitter (~0.5 deg) so the servo
 *      doesn't buzz on every frame, wearing the gear train.
 *
 *   2. Slew rate limit — cap how much the commanded servo position can
 *      change per loop, so a sudden 30 deg heading change doesn't try
 *      to teleport the servo (which it can't do anyway).
 *
 * Both behaviours combine to give the turret a "locked on" state that is
 * actually meaningful — it now means "aimed AND settled", not just
 * "tag was visible at some point this loop".
 */
public class TurretSubsystem {

    // Servo position constants
    public static final double TURRET_CENTER = 0.5;
    public static final double TURRET_MIN = 0.0;
    public static final double TURRET_MAX = 1.0;

    // goBILDA Super Speed: 90 degrees of mechanical travel maps to 0..1 servo
    public static final double DEGREES_PER_SERVO_UNIT = 90.0;

    // Don't move the turret for TX errors smaller than this — Limelight jitter
    public static final double TX_DEADBAND_DEG = 0.5;

    // Maximum servo-units of movement per update() call. With ~50 Hz loop and
    // a Super Speed servo (~0.07s/60deg = ~14 deg per 20ms loop = ~0.15 units),
    // limit to 0.04 to keep motion controlled.
    public static final double MAX_SLEW_PER_LOOP = 0.04;

    // "Aimed" threshold — turret is considered locked when within this many
    // degrees of perfect alignment.
    public static final double LOCKED_TX_THRESHOLD_DEG = 2.0;

    private final Servo servo;
    private double currentPosition = TURRET_CENTER;
    private boolean lockedOn = false;
    private double lastKnownTx = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "turretGear");
        servo.setPosition(TURRET_CENTER);
    }

    /**
     * Aim the turret toward a target with the given horizontal offset.
     *
     * @param txDegrees    horizontal angle of target from Limelight (positive = right of center)
     * @param hasTarget    true if Limelight currently has a valid target
     */
    public void aimAtTarget(double txDegrees, boolean hasTarget) {
        if (!hasTarget) {
            // Hold last commanded position; do not snap to center.
            // Snapping to center on every momentary loss-of-tag was a common
            // source of "turret pumping" in the original code.
            lockedOn = false;
            return;
        }

        lastKnownTx = txDegrees;

        // Deadband: tiny errors don't move the servo
        if (Math.abs(txDegrees) < TX_DEADBAND_DEG) {
            lockedOn = true;
            return;  // hold current position
        }

        double servoOffset = txDegrees / DEGREES_PER_SERVO_UNIT;
        double desired = TURRET_CENTER - servoOffset;
        desired = Math.max(TURRET_MIN, Math.min(TURRET_MAX, desired));

        // Slew-limit the position update
        double delta = desired - currentPosition;
        delta = Math.max(-MAX_SLEW_PER_LOOP, Math.min(MAX_SLEW_PER_LOOP, delta));
        currentPosition += delta;

        servo.setPosition(currentPosition);

        lockedOn = Math.abs(txDegrees) < LOCKED_TX_THRESHOLD_DEG;
    }

    /** Force turret back to center (used when leaving auto-aim modes). */
    public void center() {
        currentPosition = TURRET_CENTER;
        servo.setPosition(TURRET_CENTER);
        lockedOn = false;
    }

    /** Manually nudge the turret (driver-stick override). */
    public void setPosition(double position) {
        currentPosition = Math.max(TURRET_MIN, Math.min(TURRET_MAX, position));
        servo.setPosition(currentPosition);
        lockedOn = false;
    }

    /**
     * True when the turret is aimed within LOCKED_TX_THRESHOLD_DEG of the
     * target AND a target is currently visible. This is the gate to combine
     * with shooter.isReady() before firing.
     */
    public boolean isLockedOn() {
        return lockedOn;
    }

    public double getPosition()      { return currentPosition; }
    public double getLastKnownTx()    { return lastKnownTx; }
}
