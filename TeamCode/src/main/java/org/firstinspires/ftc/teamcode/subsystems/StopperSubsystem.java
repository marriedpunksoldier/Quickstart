package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Shooter.ShooterConfig;

/**
 * StopperSubsystem controls the ball-gate servo as a DISCRETE FIRE COMMAND
 * rather than the "always open while ready" pattern of the original code.
 *
 * THE BUG THIS FIXES
 * ------------------
 * BlueTeleopv2's handleStopper() opened the servo whenever both
 * limelightHasTarget AND shooterVelocityReached were true. With balls queued
 * behind the stopper, this meant the robot would dump them automatically the
 * instant conditions were met — the driver had zero shot-timing control.
 * Also caused servo chatter as readiness flickered on the tolerance edge.
 *
 * THE FIX
 * -------
 * The stopper now requires an explicit fire trigger (RT pressed) AND the
 * readiness conditions. It opens for STOPPER_OPEN_DURATION_MS then closes,
 * enforces MIN_INTER_SHOT_MS between shots, and ignores held triggers
 * (rising-edge only).
 *
 * If the hardware doesn't have a stopper servo (e.g. early prototype),
 * pass null/skip init — the methods become no-ops.
 */
public class StopperSubsystem {

    // Servo positions
    public static final double POSITION_STOP_BALL    = 0.6;
    public static final double POSITION_RELEASE_BALL = 0.2;

    private final Servo servo;             // may be null if not present
    private boolean open = false;
    private long openSinceMs = 0;
    private long lastShotMs = 0;

    /**
     * Construct from hardware map. If the servo is not configured, the
     * subsystem operates as a no-op (logged once during init).
     */
    public StopperSubsystem(HardwareMap hardwareMap) {
        Servo s = null;
        try {
            s = hardwareMap.get(Servo.class, "stopperServo");
            s.setPosition(POSITION_STOP_BALL);
        } catch (Exception ignored) {
            // No stopper hardware on this robot — methods become no-ops.
        }
        servo = s;
    }

    /**
     * Attempt to fire a single ball.
     *
     * @param firePressed  the discrete fire input (already debounced to rising-edge)
     * @param readyToFire  combined ready signal (shooter at speed AND turret locked)
     * @return true if this call actually opened the stopper (i.e. a shot was triggered)
     */
    public boolean tryFire(boolean firePressed, boolean readyToFire) {
        long now = System.currentTimeMillis();

        if (firePressed
                && readyToFire
                && !open
                && (now - lastShotMs) >= ShooterConfig.MIN_INTER_SHOT_MS) {
            open = true;
            openSinceMs = now;
            lastShotMs  = now;
            applyServo();
            return true;
        }
        return false;
    }

    /**
     * Auto-close the stopper after the configured open duration.
     * Call every loop AFTER tryFire().
     */
    public void update() {
        if (open && (System.currentTimeMillis() - openSinceMs) >= ShooterConfig.STOPPER_OPEN_DURATION_MS) {
            open = false;
            applyServo();
        }
    }

    /** Force the stopper closed immediately (e.g. for opmode stop). */
    public void forceClose() {
        open = false;
        applyServo();
    }

    public boolean isOpen() {
        return open;
    }

    public boolean isPresent() {
        return servo != null;
    }

    private void applyServo() {
        if (servo == null) return;
        servo.setPosition(open ? POSITION_RELEASE_BALL : POSITION_STOP_BALL);
    }
}
