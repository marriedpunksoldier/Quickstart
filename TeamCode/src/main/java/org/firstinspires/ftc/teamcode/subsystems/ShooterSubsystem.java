package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Shooter.ShooterConfig;

/**
 * ShooterSubsystem encapsulates ALL flywheel control logic.
 *
 * TWO-MOTOR FLYWHEEL
 * ------------------
 * The shooter is driven by two motors on opposite sides of the flywheel:
 *   shooter1 — original motor, FORWARD direction.
 *   shooter2 — opposite side, REVERSE direction (mechanically mirrored,
 *              so reversing makes a positive power command spin the
 *              flywheel the same physical way as shooter1).
 *
 * Both motors receive the same power command every loop. The control
 * loop reads velocity from shooter1's encoder only — there is one
 * flywheel, hence one speed. shooter2's encoder is read separately and
 * exposed via getMotor2RPM() purely as a health cross-check (if the two
 * encoders disagree significantly, a belt has slipped or a motor has
 * failed).
 *
 * Responsibilities:
 *   - Apply the KSV feedforward + P control law every loop
 *   - Track readiness with dwell time (no transient false-positives)
 *   - Compensate for battery voltage drift
 *   - Trip safely on overspeed
 *
 * Usage:
 *   shooter = new ShooterSubsystem(hardwareMap);
 *   shooter.setTargetRPM(2400);   // or shooter.setTargetByPower(0.5);
 *   // ... every loop:
 *   shooter.update();
 *   if (shooter.isReady()) { fire(); }
 *   // ... at end:
 *   shooter.stop();
 */
public class ShooterSubsystem {

    /**
     * If the two encoders disagree by more than this many RPM while the
     * shooter is running, something is mechanically wrong (slipped belt,
     * dead motor, unplugged encoder). Surfaced via isMotorMismatch().
     */
    public static final double MOTOR_MISMATCH_RPM = 400.0;

    private final DcMotorEx motor1;   // original — encoder used for control
    private final DcMotorEx motor2;   // opposite side — reversed
    private final VoltageSensor voltageSensor;

    // Target RPM is what the controller is aiming for; 0 means "off".
    private double targetRPM = 0.0;

    // Tracks how long we've continuously been in-tolerance, for dwell.
    // -1 means "not currently in tolerance".
    private long inToleranceSinceMs = -1;

    // Cached telemetry values (updated each call to update()).
    private double lastActualRPM = 0.0;     // from motor1
    private double lastMotor2RPM = 0.0;     // from motor2 — health check only
    private double lastMotorOutput = 0.0;
    private boolean overspeedTripped = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Motor 1 — original orientation
        motor1 = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR_1_NAME);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor 2 — opposite side, REVERSED so positive power spins the
        // flywheel the same physical direction as motor 1.
        motor2 = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR_2_NAME);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Voltage sensor: every Control Hub has one available
        VoltageSensor v = null;
        try {
            v = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {
            // If unavailable for any reason, voltage compensation is disabled.
        }
        voltageSensor = v;
    }

    // -------------------------------------------------------------------------
    // Public API — setting the target
    // -------------------------------------------------------------------------

    /** Set target RPM directly (e.g. 2400). Pass 0 to stop the shooter. */
    public void setTargetRPM(double rpm) {
        if (rpm <= 0) {
            stop();
            return;
        }
        targetRPM = rpm;
        overspeedTripped = false;
        // Don't reset dwell here — let it accumulate as the motor settles.
    }

    /** Set target as a fraction of motor free speed (e.g. 0.5 -> 3000 RPM). */
    public void setTargetByPower(double powerFraction) {
        setTargetRPM(powerFraction * ShooterConfig.MOTOR_FREE_SPEED_RPM);
    }

    /** Hard stop — disables output on both motors and clears readiness state. */
    public void stop() {
        targetRPM = 0.0;
        applyPower(0.0);
        lastMotorOutput = 0.0;
        inToleranceSinceMs = -1;
        overspeedTripped = false;
    }

    // -------------------------------------------------------------------------
    // Main update — call every loop iteration
    // -------------------------------------------------------------------------

    /**
     * Reads current velocity, computes new motor output, applies it to BOTH
     * motors, and updates the readiness state. Must be called every loop,
     * even when the shooter is "off" (so the motor command stays at 0).
     */
    public void update() {
        // Read both encoders. motor1 drives the control loop; motor2 is a
        // health cross-check.
        lastActualRPM = ticksToRPM(motor1.getVelocity());
        lastMotor2RPM = ticksToRPM(motor2.getVelocity());
        double actualRPM = lastActualRPM;

        if (targetRPM <= 0.0) {
            applyPower(0.0);
            lastMotorOutput = 0.0;
            inToleranceSinceMs = -1;
            return;
        }

        // Overspeed safety: if the flywheel is significantly overshooting,
        // disable output immediately. Protects against runaway from a
        // too-high KP or a tuning mistake. With two motors there is more
        // torque available, so this guard matters more than before.
        if (actualRPM > targetRPM * ShooterConfig.OVERSPEED_TRIP_FACTOR && targetRPM > 500) {
            overspeedTripped = true;
        }
        if (overspeedTripped) {
            applyPower(0.0);
            lastMotorOutput = 0.0;
            inToleranceSinceMs = -1;
            return;
        }

        // KSV feedforward + P
        double feedForward = ShooterConfig.KV_INITIAL * targetRPM + ShooterConfig.KS_INITIAL;
        double pTerm       = ShooterConfig.KP_INITIAL * (targetRPM - actualRPM);
        double output      = feedForward + pTerm;

        // Voltage compensation: scale output up when battery sags below 12 V
        // so the actual delivered power stays the same.
        if (voltageSensor != null) {
            double v = voltageSensor.getVoltage();
            if (v > 8.0 && v < 14.0) {           // sanity check
                output *= 12.0 / Math.max(11.0, v);
            }
        }

        output = Math.max(0.0, Math.min(1.0, output));
        applyPower(output);
        lastMotorOutput = output;

        // Readiness tracking with dwell
        boolean inTolerance = Math.abs(targetRPM - actualRPM) <= ShooterConfig.VELOCITY_TOLERANCE_RPM;
        long now = System.currentTimeMillis();
        if (inTolerance) {
            if (inToleranceSinceMs < 0) inToleranceSinceMs = now;
        } else {
            inToleranceSinceMs = -1;
        }
    }

    /** Apply the same power to both flywheel motors. */
    private void applyPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    private double ticksToRPM(double ticksPerSecond) {
        // Take abs so the readiness check works regardless of which way the
        // encoder counts. Direction problems are caught by the dedicated
        // mismatch checks below, not masked here.
        return Math.abs(ticksPerSecond) * 60.0 / ShooterConfig.TICKS_PER_REV;
    }

    // -------------------------------------------------------------------------
    // Public API — querying state
    // -------------------------------------------------------------------------

    /**
     * True when the flywheel has been within tolerance of target for at least
     * {@link ShooterConfig#READY_DWELL_MS}. This is the gate to release the
     * stopper / fire a ball.
     */
    public boolean isReady() {
        if (targetRPM <= 0 || overspeedTripped || inToleranceSinceMs < 0) return false;
        return (System.currentTimeMillis() - inToleranceSinceMs) >= ShooterConfig.READY_DWELL_MS;
    }

    /** True if the shooter is currently commanded on (regardless of readiness). */
    public boolean isRunning() {
        return targetRPM > 0 && !overspeedTripped;
    }

    /** True if the overspeed protection has tripped — call resetOverspeed() to clear. */
    public boolean isOverspeedTripped() {
        return overspeedTripped;
    }

    /** Clear an overspeed trip. Will only re-engage if conditions are still bad. */
    public void resetOverspeed() {
        overspeedTripped = false;
    }

    public double getTargetRPM()      { return targetRPM; }
    public double getActualRPM()      { return lastActualRPM; }   // motor1
    public double getMotor2RPM()      { return lastMotor2RPM; }
    public double getError()          { return targetRPM - lastActualRPM; }
    public double getMotorOutput()    { return lastMotorOutput; }

    /**
     * True if the two flywheel motors' encoders disagree by more than
     * MOTOR_MISMATCH_RPM while the shooter is running. A persistent mismatch
     * means a slipped belt, a dead motor, or an unplugged encoder — surface
     * this on telemetry so it's caught in the pit, not mid-match.
     */
    public boolean isMotorMismatch() {
        if (targetRPM <= 0 || lastMotorOutput < 0.1) return false;
        return Math.abs(lastActualRPM - lastMotor2RPM) > MOTOR_MISMATCH_RPM;
    }

    /**
     * True if motor 1 appears wired opposite to its encoder (commanded
     * positive power but reading negative velocity). Init-time diagnostic.
     */
    public boolean isDirectionMismatched() {
        if (targetRPM <= 0 || lastMotorOutput < 0.1) return false;
        return motor1.getVelocity() < -50;
    }
}
