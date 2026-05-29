package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * VerifyTune — Step 4: validate the full tuned system.
 *
 * THIS OPMODE USES THE EXACT SAME SHOOTERSUBSYSTEM YOUR MATCH CODE USES.
 * That's the whole point — if it passes here it passes in matches, because
 * there is now one and only one implementation of the control law.
 *
 * Previously, VerifyTune re-implemented the control loop locally, so a
 * passing verification said nothing about what the actual match code did.
 * That's fixed now.
 *
 * PROCEDURE
 * ---------
 *   1. Spin up (RB). Confirm the shooter reaches and holds target.
 *   2. Press A to fire the indexer for one ball. Repeat for several
 *      shots and watch the readiness flag toggle correctly.
 *   3. Press X to spin down. Confirm it recovers when spun up again.
 *   4. After 5+ shots without an out-of-tolerance event, the readout
 *      will show "READY FOR COMPETITION".
 *
 * Controls (gamepad2):
 *   right_bumper       — spin shooter ON
 *   left_bumper / X    — spin shooter OFF
 *   back               — emergency stop
 *   A                  — fire one ball via indexer
 *   right_trigger      — target RPM + RPM_STEP
 *   left_trigger       — target RPM - RPM_STEP
 */
@TeleOp(name = "Shooter - 4. Verify Tune", group = "Shooter Tuning")
public class VerifyTune extends LinearOpMode {

    // Pass thresholds (per the prior reviews)
    private static final int    MIN_SHOTS_TO_PASS         = 5;
    private static final long   MAX_SPINUP_MS             = 250;
    private static final double MIN_SESSION_SEC_TO_PASS   = 5.0;
    private static final int    MIN_SPINUP_CYCLES_TO_PASS = 2;

    private ShooterSubsystem shooter;
    private IntakeSubsystem  intake;

    private double targetRPM = ShooterConfig.TARGET_RPM;
    private boolean firing = false;
    private long fireStartMs = 0;

    // Spin-up timing
    private long spinUpStartMs = 0;
    private boolean spinUpStarted = false;
    private long latestSpinUpMs = -1;
    private boolean inSpinUpCycle = false;
    private int spinUpCycles = 0;

    // Match-readiness criteria
    private int totalShots = 0;
    private boolean everOutOfTolWhileReady = false;
    private boolean previousReady = false;

    private final ElapsedTime sessionTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime aTimer = new ElapsedTime();
    private final ElapsedTime rtTimer = new ElapsedTime();
    private final ElapsedTime ltTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake  = new IntakeSubsystem(hardwareMap);

        telemetry.addLine("=== VERIFY TUNE ===");
        telemetry.addLine("Using ShooterSubsystem — same code your matches run.");
        telemetry.addLine();
        telemetry.addLine("Active coefficients:");
        telemetry.addData("  KS", "%.4f", ShooterConfig.KS_INITIAL);
        telemetry.addData("  KV", "%.6f", ShooterConfig.KV_INITIAL);
        telemetry.addData("  KP", "%.4f", ShooterConfig.KP_INITIAL);
        telemetry.addLine();
        telemetry.addLine("If these are wrong, STOP and update ShooterConfig.");
        telemetry.addLine("Press PLAY to confirm and begin.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        sessionTimer.reset();

        while (opModeIsActive()) {
            handleInputs();

            shooter.update();

            // Track spin-up timing
            if (shooter.isRunning() && !spinUpStarted) {
                spinUpStarted = true;
                spinUpStartMs = System.currentTimeMillis();
                inSpinUpCycle = true;
            }
            if (inSpinUpCycle && shooter.isReady()) {
                latestSpinUpMs = System.currentTimeMillis() - spinUpStartMs
                                  - ShooterConfig.READY_DWELL_MS;
                if (latestSpinUpMs < 0) latestSpinUpMs = 0;
                inSpinUpCycle = false;
                spinUpCycles++;
            }
            if (!shooter.isRunning()) {
                spinUpStarted = false;
                inSpinUpCycle = false;
            }

            // Watch for "I claimed ready, then fell out of tolerance" — bad sign
            boolean nowReady = shooter.isReady();
            if (previousReady && !nowReady && shooter.isRunning()) {
                // We were ready, now we're not, and the shooter is still commanded on.
                // If we weren't firing, that's an out-of-tolerance event.
                if (!firing && System.currentTimeMillis() - fireStartMs > 1000) {
                    everOutOfTolWhileReady = true;
                }
            }
            previousReady = nowReady;

            // Feed pulse during a shot — same code path as match autonomous
            if (firing) {
                if (System.currentTimeMillis() - fireStartMs < ShooterConfig.INDEXER_ON_TIME_MS) {
                    intake.feedShooter();
                } else {
                    intake.stop();
                    firing = false;
                    totalShots++;
                }
            }

            if (telemetryTimer.milliseconds() > ShooterConfig.TELEMETRY_UPDATE_MS) {
                updateTelemetry();
                telemetryTimer.reset();
            }
        }

        shooter.stop();
        intake.stop();
    }

    private void handleInputs() {
        if (gamepad2.back) {
            shooter.stop();
            intake.stop();
            firing = false;
            return;
        }

        if (gamepad2.right_bumper) {
            shooter.setTargetRPM(targetRPM);
        } else if (gamepad2.left_bumper || gamepad2.x) {
            shooter.stop();
        }

        if (debounced(gamepad2.right_trigger > 0.2, rtTimer)) {
            targetRPM = Math.min(ShooterConfig.MOTOR_FREE_SPEED_RPM,
                                 targetRPM + ShooterConfig.RPM_STEP);
        }
        if (debounced(gamepad2.left_trigger > 0.2, ltTimer)) {
            targetRPM = Math.max(0.0, targetRPM - ShooterConfig.RPM_STEP);
        }

        // Fire a ball
        if (debounced(gamepad2.a, aTimer) && shooter.isReady() && !firing) {
            firing = true;
            fireStartMs = System.currentTimeMillis();
        }
    }

    private boolean debounced(boolean pressed, ElapsedTime timer) {
        if (pressed && timer.milliseconds() > ShooterConfig.BUTTON_DEBOUNCE_MS) {
            timer.reset();
            return true;
        }
        return false;
    }

    private void updateTelemetry() {
        telemetry.addLine("=== VERIFY TUNE ===");
        telemetry.addData("Session time", "%.1f s", sessionTimer.seconds());
        telemetry.addLine();

        telemetry.addData("Target RPM",  "%.0f", shooter.getTargetRPM());
        telemetry.addData("Actual RPM",  "%.0f", shooter.getActualRPM());
        telemetry.addData("Motor out",   "%.3f", shooter.getMotorOutput());
        telemetry.addData("Ready",       shooter.isReady() ? "YES" : "no");
        if (shooter.isOverspeedTripped()) {
            telemetry.addLine(">>> OVERSPEED TRIPPED <<<");
        }
        telemetry.addLine();

        telemetry.addLine("Pass criteria:");
        telemetry.addData("  Spin-up time (latest)", latestSpinUpMs >= 0
                ? String.format("%d ms  %s", latestSpinUpMs,
                        latestSpinUpMs <= MAX_SPINUP_MS ? "PASS" : "TOO SLOW")
                : "not yet measured");
        telemetry.addData("  Spin-up cycles",       spinUpCycles + " (need " + MIN_SPINUP_CYCLES_TO_PASS + ")");
        telemetry.addData("  Shots fired",          totalShots + " (need " + MIN_SHOTS_TO_PASS + ")");
        telemetry.addData("  Out-of-tol events",    everOutOfTolWhileReady ? "FAIL" : "none");
        telemetry.addLine();

        boolean allPass = !everOutOfTolWhileReady
                && latestSpinUpMs >= 0 && latestSpinUpMs <= MAX_SPINUP_MS
                && spinUpCycles >= MIN_SPINUP_CYCLES_TO_PASS
                && totalShots >= MIN_SHOTS_TO_PASS
                && sessionTimer.seconds() >= MIN_SESSION_SEC_TO_PASS;

        telemetry.addLine(allPass
                ? "*** READY FOR COMPETITION ***"
                : "not yet ready — keep testing");
        telemetry.update();
    }
}
