package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * KPTune — Step 3 of the shooter tuning pipeline.
 *
 * GOAL
 * ----
 * With KS and KV fixed, find the smallest KP that produces fast recovery
 * after a ball is fired (RPM dip), without oscillating through target.
 *
 * SIMULATING A SHOT
 * -----------------
 * Press A to briefly load the motor (in a real setup, press A while
 * physically dragging the flywheel by hand, or have a teammate flick the
 * wheel). Observe the recovery curve on telemetry. Adjust KP and repeat.
 *
 * KEY FIXES vs the original
 * --------------------------
 *  1. Readiness uses the FINAL target with dwell, not the in-flight
 *     ramp value. This is the same fix as KVTune.
 *  2. Peak error tracking measures across the entire shot+recovery
 *     window, not reset prematurely.
 *  3. Overspeed protection — important for KP since cranking KP causes
 *     oscillation that briefly overshoots target.
 *  4. Motor mode set once in init.
 *
 * Controls (gamepad2):
 *   right_bumper       — spin ON
 *   left_bumper / back — spin OFF / emergency stop
 *   dpad_up/down       — adjust KP by KP_STEP
 *   right_trigger      — target RPM + RPM_STEP
 *   left_trigger       — target RPM - RPM_STEP
 *   A                  — simulate a shot (full power for 80ms, observe dip)
 */
@TeleOp(name = "Shooter - 3. KP Tune (Proportional Recovery)", group = "Shooter Tuning")
public class KPTune extends LinearOpMode {

    private FlywheelMotors flywheel;
    private final double ks = ShooterConfig.KS_INITIAL;
    private final double kv = ShooterConfig.KV_INITIAL;
    private double kp = ShooterConfig.KP_INITIAL;
    private double targetRPM = Math.min(ShooterConfig.TARGET_RPM, 2500.0);
    private boolean flywheelOn = false;

    private double currentRampedRPM = 0.0;
    private final ElapsedTime rampTimer = new ElapsedTime();

    // Readiness with dwell
    private long inToleranceSinceMs = -1;
    private boolean ready = false;

    // Peak error tracking (across firing windows)
    private double peakError = 0.0;
    private long peakErrorClearMs = 0;

    // Simulated shot
    private boolean simulatingShot = false;
    private long shotStartMs = 0;
    private static final long SHOT_DURATION_MS = 80;

    private final ElapsedTime dpadUpTimer   = new ElapsedTime();
    private final ElapsedTime dpadDownTimer = new ElapsedTime();
    private final ElapsedTime rtTimer       = new ElapsedTime();
    private final ElapsedTime ltTimer       = new ElapsedTime();
    private final ElapsedTime aTimer        = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        flywheel = new FlywheelMotors(hardwareMap);

        telemetry.addLine("KP Tune: confirm KS and KV are set from prior steps.");
        telemetry.addLine("Press PLAY when ready.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        rampTimer.reset();

        while (opModeIsActive()) {
            handleControls();

            double dt = rampTimer.seconds();
            rampTimer.reset();
            currentRampedRPM = rampToward(currentRampedRPM, flywheelOn ? targetRPM : 0.0, dt);

            double actualRPM = flywheel.getAbsRPM();

            // Full control law: feedforward + P
            double ff = kv * currentRampedRPM + ks;
            double pTerm = kp * (targetRPM - actualRPM);
            double output = Range.clip(ff + pTerm, 0.0, 1.0);

            // Simulated shot: briefly cut power to mimic load
            if (simulatingShot) {
                output = 0;
                if (System.currentTimeMillis() - shotStartMs > SHOT_DURATION_MS) {
                    simulatingShot = false;
                    peakError = 0;  // start tracking peak from end-of-shot
                    peakErrorClearMs = System.currentTimeMillis();
                }
            }

            // Overspeed safety
            if (flywheelOn && actualRPM > targetRPM * ShooterConfig.OVERSPEED_TRIP_FACTOR
                    && currentRampedRPM > 500) {
                flywheelOn = false;
                output = 0;
            }

            flywheel.setPower(flywheelOn ? output : 0);

            // Readiness with dwell — vs FINAL target, not ramp
            boolean rampComplete = !flywheelOn
                    || Math.abs(currentRampedRPM - targetRPM) < 1.0;
            boolean inTolerance = rampComplete && flywheelOn
                    && Math.abs(targetRPM - actualRPM) <= ShooterConfig.VELOCITY_TOLERANCE_RPM;
            if (inTolerance) {
                if (inToleranceSinceMs < 0) inToleranceSinceMs = System.currentTimeMillis();
                ready = (System.currentTimeMillis() - inToleranceSinceMs)
                        >= ShooterConfig.READY_DWELL_MS;
            } else {
                inToleranceSinceMs = -1;
                ready = false;
            }

            // Track peak error in the post-shot window (3-second sliding)
            double err = Math.abs(targetRPM - actualRPM);
            if (System.currentTimeMillis() - peakErrorClearMs < 3000) {
                peakError = Math.max(peakError, err);
            }

            if (telemetryTimer.milliseconds() > ShooterConfig.TELEMETRY_UPDATE_MS) {
                updateTelemetry(actualRPM, output, rampComplete);
                telemetryTimer.reset();
            }
        }

        flywheel.setPower(0);
    }

    private void handleControls() {
        if (gamepad2.left_bumper || gamepad2.back) {
            flywheelOn = false;
        } else if (gamepad2.right_bumper) {
            flywheelOn = true;
        }

        if (debounced(gamepad2.dpad_up, dpadUpTimer)) {
            kp += ShooterConfig.KP_STEP;
        }
        if (debounced(gamepad2.dpad_down, dpadDownTimer)) {
            kp = Math.max(0.0, kp - ShooterConfig.KP_STEP);
        }

        if (debounced(gamepad2.right_trigger > 0.2, rtTimer)) {
            targetRPM = Math.min(ShooterConfig.MOTOR_FREE_SPEED_RPM,
                    targetRPM + ShooterConfig.RPM_STEP);
        }
        if (debounced(gamepad2.left_trigger > 0.2, ltTimer)) {
            targetRPM = Math.max(0.0, targetRPM - ShooterConfig.RPM_STEP);
        }

        // Simulated shot — briefly cut power and watch recovery
        if (debounced(gamepad2.a, aTimer) && flywheelOn && ready && !simulatingShot) {
            simulatingShot = true;
            shotStartMs = System.currentTimeMillis();
        }
    }

    private boolean debounced(boolean pressed, ElapsedTime timer) {
        if (pressed && timer.milliseconds() > ShooterConfig.BUTTON_DEBOUNCE_MS) {
            timer.reset();
            return true;
        }
        return false;
    }

    private double rampToward(double current, double desired, double dt) {
        if (ShooterConfig.RAMP_RATE_RPM_PER_SEC <= 0) return desired;
        double maxDelta = ShooterConfig.RAMP_RATE_RPM_PER_SEC * dt;
        return current + Range.clip(desired - current, -maxDelta, maxDelta);
    }

    private void updateTelemetry(double actualRPM, double output, boolean rampComplete) {
        telemetry.addLine("=== KP TUNE (Step 3 of 3) ===");
        telemetry.addData("Flywheel",   flywheelOn ? "ON" : "OFF");
        telemetry.addData("Ramp",       rampComplete ? "SETTLED" : "ramping...");
        telemetry.addData("Ready",      ready ? "YES (dwell met)" : "no");
        if (simulatingShot) {
            telemetry.addLine(">> SIMULATING SHOT (power=0) <<");
        }
        telemetry.addLine();

        telemetry.addData("Target",     "%.1f RPM", targetRPM);
        telemetry.addData("Actual",     "%.1f RPM", actualRPM);
        telemetry.addData("Error",      "%.1f", targetRPM - actualRPM);
        telemetry.addData("Peak err (3s window)", "%.1f", peakError);
        telemetry.addLine();

        telemetry.addData("KS (fixed)", "%.4f", ks);
        telemetry.addData("KV (fixed)", "%.6f", kv);
        telemetry.addData("KP (tuning)","%.4f", kp);
        telemetry.addData("Output",     "%.3f", output);
        telemetry.addLine();

        telemetry.addLine("Procedure:");
        telemetry.addLine("  1. Spin up (RB). Wait for ready.");
        telemetry.addLine("  2. Press A to simulate shot.");
        telemetry.addLine("  3. Observe peak error and recovery time.");
        telemetry.addLine("  4. Raise KP (dpad_up) until peak error drops,");
        telemetry.addLine("     but stop if you see oscillation.");
        telemetry.addLine();
        telemetry.addLine("Copy/paste:");
        telemetry.addData("=>", "public static final double KP_INITIAL = %.4f;", kp);
        telemetry.update();
    }
}
