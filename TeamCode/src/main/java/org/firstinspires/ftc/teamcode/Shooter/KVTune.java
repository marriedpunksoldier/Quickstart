package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * KVTune — Step 2 of the shooter tuning pipeline.
 *
 * GOAL
 * ----
 * With KS fixed and KP=0, find the KV value that produces target RPM
 * at steady state using feedforward alone.
 *
 * KEY FIXES vs the original
 * --------------------------
 *  1. Error display shows error against the FINAL TARGET RPM, not the
 *     in-flight ramp value. The previous version computed error vs the
 *     ramping setpoint, which means during the ~1s ramp the error
 *     reading was always near zero regardless of KV — causing tuners
 *     to think they were done while the actual physical RPM was wrong.
 *  2. Tuning guidance is suppressed until the ramp has completed.
 *  3. Motor mode is set once in init, not every loop.
 *  4. Coarse adjustment with L3 (left stick button) for 4x step size,
 *     so you can get into the right neighbourhood without 50 button
 *     presses.
 *  5. Overspeed safety trip.
 *  6. Both bumpers + back as a deadman stop.
 *
 * Controls (gamepad2):
 *   right_bumper          — spin ON (left bumper takes priority)
 *   left_bumper, back     — spin OFF / emergency stop
 *   dpad_up/down          — adjust KV by KV_STEP
 *   left_stick_button +   — coarse adjustment (4x step)
 *   right_trigger         — target RPM +RPM_STEP
 *   left_trigger          — target RPM -RPM_STEP
 */
@TeleOp(name = "Shooter - 2. KV Tune (Velocity Feedforward)", group = "Shooter Tuning")
public class KVTune extends LinearOpMode {

    private static final double COARSE_KV_MULTIPLIER = 4.0;

    private FlywheelMotors flywheel;
    private double kv = ShooterConfig.KV_INITIAL;
    private final double ks = ShooterConfig.KS_INITIAL;
    private double targetRPM = Math.min(ShooterConfig.TARGET_RPM, 2000.0);  // safe start
    private boolean flywheelOn = false;

    private double currentRampedRPM = 0.0;
    private final ElapsedTime rampTimer = new ElapsedTime();

    private final ElapsedTime dpadUpTimer   = new ElapsedTime();
    private final ElapsedTime dpadDownTimer = new ElapsedTime();
    private final ElapsedTime rtTimer       = new ElapsedTime();
    private final ElapsedTime ltTimer       = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addLine("Initialising KV Tune...");
        telemetry.update();

        flywheel = new FlywheelMotors(hardwareMap);

        telemetry.addLine("Confirm KS_INITIAL is set from Step 1.");
        telemetry.addLine("Press PLAY when ready.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        rampTimer.reset();

        while (opModeIsActive()) {
            handleSpinControl();
            handleKVAdjustment();
            handleRPMAdjustment();

            // Ramp the commanded target
            double dt = rampTimer.seconds();
            rampTimer.reset();
            currentRampedRPM = rampToward(currentRampedRPM, flywheelOn ? targetRPM : 0.0, dt);

            // Feedforward only — KP=0 during this step
            double ff = Range.clip(kv * currentRampedRPM + ks, 0.0, 1.0);
            flywheel.setPower(flywheelOn ? ff : 0.0);

            // Read actual velocity (signed for direction sanity check)
            double signedRPM = flywheel.getRPM();
            double actualRPM = Math.abs(signedRPM);

            // Overspeed safety
            if (flywheelOn && actualRPM > targetRPM * ShooterConfig.OVERSPEED_TRIP_FACTOR
                    && currentRampedRPM > 500) {
                flywheelOn = false;
            }

            if (telemetryTimer.milliseconds() > ShooterConfig.TELEMETRY_UPDATE_MS) {
                updateTelemetry(actualRPM, signedRPM, ff);
                telemetryTimer.reset();
            }
        }

        flywheel.setPower(0);
    }

    private void handleSpinControl() {
        // Left/back wins on tie — emergency stop is unambiguous
        if (gamepad2.left_bumper || gamepad2.back) {
            flywheelOn = false;
        } else if (gamepad2.right_bumper) {
            flywheelOn = true;
        }
    }

    private void handleKVAdjustment() {
        double step = gamepad2.left_stick_button
                ? ShooterConfig.KV_STEP * COARSE_KV_MULTIPLIER
                : ShooterConfig.KV_STEP;

        if (debounced(gamepad2.dpad_up, dpadUpTimer)) {
            kv += step;
        }
        if (debounced(gamepad2.dpad_down, dpadDownTimer)) {
            kv = Math.max(0.0, kv - step);
        }
    }

    private void handleRPMAdjustment() {
        if (debounced(gamepad2.right_trigger > 0.2, rtTimer)) {
            targetRPM = Math.min(ShooterConfig.MOTOR_FREE_SPEED_RPM,
                    targetRPM + ShooterConfig.RPM_STEP);
        }
        if (debounced(gamepad2.left_trigger > 0.2, ltTimer)) {
            targetRPM = Math.max(0.0, targetRPM - ShooterConfig.RPM_STEP);
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

    private void updateTelemetry(double actualRPM, double signedRPM, double appliedPower) {
        boolean rampComplete = !flywheelOn
                || Math.abs(currentRampedRPM - targetRPM) < 1.0;
        double trueError = targetRPM - actualRPM;

        telemetry.addLine("=== KV TUNE (Step 2 of 3) ===");
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Ramp", rampComplete ? "SETTLED" : "ramping...");
        telemetry.addLine();

        telemetry.addData("Target RPM (final)", "%.1f", targetRPM);
        telemetry.addData("Ramped RPM",         "%.1f", currentRampedRPM);
        telemetry.addData("Actual RPM",         "%.1f", actualRPM);
        telemetry.addLine();

        if (!flywheelOn) {
            telemetry.addLine("Press RB to spin up.");
        } else if (!rampComplete) {
            telemetry.addLine("Waiting for ramp to complete before tuning.");
        } else if (trueError > 50) {
            telemetry.addData("Error", "%.1f  >> KV too LOW (dpad_up)", trueError);
        } else if (trueError < -50) {
            telemetry.addData("Error", "%.1f  >> KV too HIGH (dpad_down)", trueError);
        } else {
            telemetry.addData("Error", "%.1f  >> WITHIN 50 RPM. Save KV.", trueError);
        }

        if (signedRPM < -50 && flywheelOn) {
            telemetry.addLine(">>> MOTOR DIRECTION REVERSED <<<");
        }
        telemetry.addLine();

        telemetry.addData("KS (fixed)",       "%.4f", ks);
        telemetry.addData("KV (tuning)",      "%.6f", kv);
        telemetry.addData("Applied power",    "%.4f", appliedPower);
        telemetry.addLine();

        telemetry.addLine("Controls: RB=on, LB/back=off, dpad=adjust KV");
        telemetry.addLine("Hold L3 stick button for coarse (4x) step");
        telemetry.addLine("Triggers = adjust target RPM");
        telemetry.addLine();
        telemetry.addLine("Copy/paste:");
        telemetry.addData("=>", "public static final double KV_INITIAL = %.6f;", kv);
        telemetry.update();
    }
}
