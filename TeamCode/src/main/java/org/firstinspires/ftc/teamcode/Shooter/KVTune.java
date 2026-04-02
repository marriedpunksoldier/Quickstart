package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * KVTune.java — Step 2 of 3 in the shooter tuning pipeline.
 *
 * PURPOSE
 * -------
 * Tune KV (velocity feedforward) so the flywheel reaches TARGET_RPM at
 * steady state using feedforward alone — no KP correction involved yet.
 *
 * WHY THIS MATTERS (from the video)
 * ----------------------------------
 * "KV keeps us at the right speed."
 * "KV is the minimum value we have to send to be able to achieve the set point."
 *
 * The full feedforward equation is:
 *   feedForward = (KV * targetRPM) + KS
 *
 * With a well-tuned KV, the motor output power is pre-calculated correctly
 * for the desired RPM — meaning the P controller (KP) only has to handle
 * small transient disturbances rather than carrying the steady-state load.
 * This is what enables the video's 0.24-second spin-up time.
 *
 * CONTROL LAW USED HERE (KP = 0)
 * --------------------------------
 *   motorPower = (KV * targetRPM) + KS
 *
 * This OpMode runs open-loop (RUN_WITHOUT_ENCODER) and applies power directly.
 * We bypass the SDK's built-in velocity PID entirely so we can see what KV
 * alone achieves without any hidden correction.
 *
 * PRE-CONDITION
 * -------------
 * KS must already be tuned.  Paste it into ShooterConfig.KS_INITIAL before
 * running this OpMode.
 *
 * TUNING PROCEDURE
 * ----------------
 *   1. Press gamepad2 RIGHT_BUMPER to spin the flywheel up.
 *   2. Watch "Actual RPM" stabilize (allow ~2 seconds to settle).
 *   3. Compare Actual RPM vs Target RPM:
 *      - Actual < Target (positive error) → KV too LOW  → press dpad_up
 *      - Actual > Target (negative error) → KV too HIGH → press dpad_down
 *   4. Make one adjustment, wait 2 seconds to settle, then re-evaluate.
 *   5. Goal: |Velocity Error| < 50 RPM at steady state with KP = 0.
 *   6. Copy the final KV value into ShooterConfig.KV_INITIAL, then run KPTune.java.
 *
 * CONTROLS (gamepad2 only)
 * -------------------------
 *   right_bumper              → spin flywheel ON
 *   left_bumper               → spin flywheel OFF
 *   dpad_up                   → increase KV by KV_STEP
 *   dpad_down                 → decrease KV by KV_STEP
 *   right_trigger (>0.2)      → increase target RPM by RPM_STEP
 *   left_trigger  (>0.2)      → decrease target RPM by RPM_STEP
 */
@TeleOp(name = "Shooter — 2. KV Tune  (Velocity Feedforward)", group = "Shooter Tuning")
public class KVTune extends LinearOpMode {

    private DcMotorEx flywheelMotor;

    // Live-tunable values
    private double kv        = ShooterConfig.KV_INITIAL;
    private double ks        = ShooterConfig.KS_INITIAL;  // read-only here; set in Step 1
    private double targetRPM = ShooterConfig.TARGET_RPM;
    private boolean flywheelOn = false;

    // Velocity ramp
    private double currentRampedRPM = 0.0;
    private final ElapsedTime rampTimer = new ElapsedTime();

    // Debounce timers
    private final ElapsedTime dpadUpTimer   = new ElapsedTime();
    private final ElapsedTime dpadDownTimer = new ElapsedTime();
    private final ElapsedTime rtTimer       = new ElapsedTime();
    private final ElapsedTime ltTimer       = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final long DEBOUNCE_MS = 200;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing KV Tune...");
        telemetry.update();

        flywheelMotor = initFlywheelMotor();

        telemetry.addLine("✓ Flywheel motor ready.");
        telemetry.addLine("Confirm KS is set in ShooterConfig before proceeding.");
        telemetry.addLine("Press PLAY to begin.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        rampTimer.reset();

        while (opModeIsActive()) {

            // --- Handle controls ---
            handleSpinToggle();
            handleKVAdjustment();
            handleRPMAdjustment();

            // --- Ramp the target RPM (avoids current spikes on spin-up) ---
            currentRampedRPM = rampRPM(currentRampedRPM, flywheelOn ? targetRPM : 0.0);

            // --- Compute feedforward power (KP = 0 during this step) ---
            // feedForward = KV * targetRPM + KS
            // Clamp to [0.0, 1.0] — motor power fraction
            double feedForwardPower = Range.clip((kv * currentRampedRPM) + ks, 0.0, 1.0);

            // Apply open-loop power directly — no SDK velocity PID involved
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor.setPower(flywheelOn ? feedForwardPower : 0.0);

            // --- Read actual velocity ---
            double actualVelocityTPS = flywheelMotor.getVelocity();
            double actualRPM         = ticksPerSecondToRPM(Math.abs(actualVelocityTPS));
            double errorRPM          = currentRampedRPM - actualRPM;

            // --- Telemetry ---
            if (telemetryTimer.milliseconds() > ShooterConfig.TELEMETRY_UPDATE_MS) {
                updateTelemetry(actualRPM, currentRampedRPM, errorRPM, feedForwardPower);
                telemetryTimer.reset();
            }
        }

        shutdown();
    }

    // -------------------------------------------------------------------------
    // Control handlers
    // -------------------------------------------------------------------------

    private void handleSpinToggle() {
        if (gamepad2.right_bumper) flywheelOn = true;
        if (gamepad2.left_bumper)  flywheelOn = false;
    }

    private void handleKVAdjustment() {
        if (gamepad2.dpad_up && dpadUpTimer.milliseconds() > DEBOUNCE_MS) {
            kv += ShooterConfig.KV_STEP;
            dpadUpTimer.reset();
        }
        if (gamepad2.dpad_down && dpadDownTimer.milliseconds() > DEBOUNCE_MS) {
            kv = Math.max(0.0, kv - ShooterConfig.KV_STEP);
            dpadDownTimer.reset();
        }
    }

    private void handleRPMAdjustment() {
        if (gamepad2.right_trigger > 0.2 && rtTimer.milliseconds() > DEBOUNCE_MS) {
            targetRPM = Math.min(ShooterConfig.MOTOR_FREE_SPEED_RPM, targetRPM + ShooterConfig.RPM_STEP);
            rtTimer.reset();
        }
        if (gamepad2.left_trigger > 0.2 && ltTimer.milliseconds() > DEBOUNCE_MS) {
            targetRPM = Math.max(0.0, targetRPM - ShooterConfig.RPM_STEP);
            ltTimer.reset();
        }
    }

    // -------------------------------------------------------------------------
    // Velocity ramp
    // -------------------------------------------------------------------------

    /**
     * Smoothly ramp current toward desired RPM at RAMP_RATE_RPM_PER_SEC.
     * Prevents large current spikes from causing a Control Hub brownout.
     */
    private double rampRPM(double current, double desired) {
        double elapsed = rampTimer.seconds();
        rampTimer.reset();
        if (ShooterConfig.RAMP_RATE_RPM_PER_SEC <= 0) return desired;
        double maxDelta = ShooterConfig.RAMP_RATE_RPM_PER_SEC * elapsed;
        return current + Range.clip(desired - current, -maxDelta, maxDelta);
    }

    // -------------------------------------------------------------------------
    // Conversions
    // -------------------------------------------------------------------------

    private double ticksPerSecondToRPM(double tps) {
        return tps * 60.0 / ShooterConfig.TICKS_PER_REV;
    }

    // -------------------------------------------------------------------------
    // Hardware init
    // -------------------------------------------------------------------------

    private DcMotorEx initFlywheelMotor() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Start in open-loop so we don't get SDK PID interference during KV tuning
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void updateTelemetry(double actualRPM, double targetRPMNow,
                                  double errorRPM, double feedForwardPower) {
        telemetry.addLine("=== KV TUNE — Velocity Feedforward ===");
        telemetry.addLine("(Step 2 of 3 — KP = 0, feedforward only)");
        telemetry.addLine();
        telemetry.addData("Flywheel", flywheelOn ? "ON (spinning)" : "OFF");
        telemetry.addLine();
        telemetry.addLine("--- Velocity ---");
        telemetry.addData("Target RPM",       "%.1f", targetRPMNow);
        telemetry.addData("Actual RPM",       "%.1f", actualRPM);
        telemetry.addData("Error RPM",        "%.1f  %s",
                errorRPM,
                errorRPM > 50  ? "↑ KV too LOW  → press dpad_up"   :
                errorRPM < -50 ? "↓ KV too HIGH → press dpad_down"  :
                                 "✓ within 50 RPM — good!");
        telemetry.addLine();
        telemetry.addLine("--- Active Coefficients ---");
        telemetry.addData("KS",              "%.4f  (fixed — from Step 1)", ks);
        telemetry.addData("KV",              "%.6f  ◄ TUNING NOW",          kv);
        telemetry.addData("KP",              "0.000000  (not used in Step 2)");
        telemetry.addLine();
        telemetry.addData("Applied Power",   "%.4f  = KV×RPM + KS", feedForwardPower);
        telemetry.addLine();
        telemetry.addLine("--- Goal RPM (adjustable) ---");
        telemetry.addData("Configured Target", "%.1f RPM", targetRPM);
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("right_bumper → spin ON  |  left_bumper → spin OFF");
        telemetry.addLine("dpad_up/down → adjust KV ± " + ShooterConfig.KV_STEP);
        telemetry.addLine("triggers     → adjust target RPM ± " + ShooterConfig.RPM_STEP);
        telemetry.addLine();
        telemetry.addLine("When |Error| < 50 RPM at steady state:");
        telemetry.addLine("Copy KV above → ShooterConfig.KV_INITIAL, then run KPTune.");
        telemetry.update();
    }

    // -------------------------------------------------------------------------
    // Shutdown
    // -------------------------------------------------------------------------

    private void shutdown() {
        if (flywheelMotor != null) {
            flywheelMotor.setPower(0);
        }
    }
}
