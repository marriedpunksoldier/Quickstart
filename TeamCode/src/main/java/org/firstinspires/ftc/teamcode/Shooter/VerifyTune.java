package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * VerifyTune.java — Step 4 of 4 in the shooter tuning pipeline.
 *
 * PURPOSE
 * -------
 * Full system validation at match-ready conditions using the final tuned
 * KS + KV + KP (PVS) controller.  Confirms the system meets the goals
 * demonstrated in the tuning video before the robot goes to competition.
 *
 * SUCCESS CRITERIA (from video)
 * --------------------------------
 * ✓ Flywheel spins up to 1500 RPM in under 0.25 seconds
 *   (video baseline was 1.2 s — properly tuned KV reduces this dramatically)
 * ✓ Velocity stays within ± VELOCITY_TOLERANCE_RPM during burst and auto-fire
 * ✓ Peak error during firing is small and short-lived
 * ✓ No RPM oscillation above or below target after shots
 *
 * VALIDATION CHECKLIST
 * --------------------
 *   □ Spin up → reach 1500 RPM — check spin-up time on telemetry
 *   □ Fire burst (3 balls) → verify recovery after each ball
 *   □ Hold auto-fire for 3+ seconds → verify sustained accuracy
 *   □ Spin down, spin up again → verify repeatable spin-up time
 *   □ "Competition Ready" shows ✓✓✓ on telemetry
 *
 * PRE-CONDITIONS
 * --------------
 * ShooterConfig must contain FINAL tuned values for KS, KV, and KP.
 *
 * CONTROLS (gamepad2 only — matches operator control layout)
 * -----------------------------------------------------------
 *   right_bumper         → spin flywheel ON
 *   left_bumper          → spin flywheel OFF
 *   right_trigger (>0.2) → auto-fire (continuous while held)
 *   x button             → burst fire (BURST_COUNT balls)
 *   dpad_right           → increase target RPM by RPM_STEP (final sanity check)
 *   dpad_left            → decrease target RPM by RPM_STEP
 */
@TeleOp(name = "Shooter — 4. Verify Tune", group = "Shooter Tuning")
public class VerifyTune extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotorEx flywheelMotor;
    private DcMotor   indexerMotor;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    private double  targetRPM       = ShooterConfig.TARGET_RPM;
    private boolean flywheelOn      = false;
    private double  currentRampedRPM = 0.0;
    private final ElapsedTime rampTimer = new ElapsedTime();

    // Flywheel ready interlock
    private boolean inTolerance   = false;
    private boolean flywheelReady = false;
    private final ElapsedTime readyDwellTimer = new ElapsedTime();

    // Spin-up timing measurement
    private boolean wasOff          = true;    // was flywheel off last loop?
    private boolean spinUpComplete  = false;
    private double  spinUpTimeMs    = 0.0;
    private final ElapsedTime spinUpTimer = new ElapsedTime();

    // Indexer state machine
    private enum IndexerState { IDLE, BURST_FIRING, AUTO_FIRING }
    private IndexerState indexerState   = IndexerState.IDLE;
    private int   burstBallsRemaining   = 0;
    private boolean indexerPhaseOn      = false;
    private final ElapsedTime indexerTimer = new ElapsedTime();

    // Verification statistics
    private int    burstShotsFired   = 0;
    private int    autoShotCycles    = 0;
    private double peakErrorRPM      = 0.0;
    private boolean everOutOfTol     = false;
    private final ElapsedTime sessionTimer = new ElapsedTime();

    // Button debounce
    private final ElapsedTime xButtonTimer   = new ElapsedTime();
    private final ElapsedTime dpadRightTimer = new ElapsedTime();
    private final ElapsedTime dpadLeftTimer  = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final long DEBOUNCE_MS = 200;

    // -------------------------------------------------------------------------
    // Main OpMode
    // -------------------------------------------------------------------------
    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing Verify Tune...");
        telemetry.update();

        flywheelMotor = initFlywheelMotor();
        indexerMotor  = initIndexerMotor();

        telemetry.addLine("✓ All motors initialized.");
        telemetry.addLine("Confirm ShooterConfig has FINAL tuned KS, KV, KP.");
        telemetry.addLine("Press PLAY to begin validation.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        rampTimer.reset();
        sessionTimer.reset();

        while (opModeIsActive()) {

            // --- Controls ---
            handleSpinToggle();
            handleRPMAdjustment();

            // --- Velocity ramp ---
            boolean spinningUp = flywheelOn && !wasOff;
            if (flywheelOn && wasOff) {
                // Flywheel just turned on — start spin-up timer
                spinUpTimer.reset();
                spinUpComplete = false;
                wasOff = false;
            } else if (!flywheelOn) {
                wasOff = true;
                spinUpComplete = false;
            }

            currentRampedRPM = rampRPM(currentRampedRPM, flywheelOn ? targetRPM : 0.0);

            // --- PVS control law ---
            double actualVelocityTPS = flywheelMotor.getVelocity();
            double actualRPM         = ticksPerSecondToRPM(Math.abs(actualVelocityTPS));
            double errorRPM          = currentRampedRPM - actualRPM;

            double feedForward = (ShooterConfig.KV_INITIAL * currentRampedRPM) + ShooterConfig.KS_INITIAL;
            double pTerm       = ShooterConfig.KP_INITIAL * errorRPM;
            double motorPower  = Range.clip(feedForward + pTerm, 0.0, 1.0);

            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor.setPower(flywheelOn ? motorPower : 0.0);

            // --- Spin-up time measurement ---
            if (flywheelOn && !spinUpComplete
                    && Math.abs(errorRPM) <= ShooterConfig.VELOCITY_TOLERANCE_RPM) {
                spinUpTimeMs   = spinUpTimer.milliseconds();
                spinUpComplete = true;
            }

            // --- Flywheel ready interlock ---
            updateFlywheelReadiness(errorRPM);

            // --- Statistics (while flywheel is commanded on) ---
            if (flywheelOn && currentRampedRPM > 100) {
                double absErr = Math.abs(errorRPM);
                if (absErr > peakErrorRPM)                              peakErrorRPM = absErr;
                if (absErr > ShooterConfig.VELOCITY_TOLERANCE_RPM)      everOutOfTol = true;
            }

            // --- Indexer ---
            handleIndexerInput();
            tickIndexerStateMachine();

            // --- Telemetry ---
            if (telemetryTimer.milliseconds() > ShooterConfig.TELEMETRY_UPDATE_MS) {
                updateTelemetry(actualRPM, currentRampedRPM, errorRPM, motorPower);
                telemetryTimer.reset();
            }
        }

        shutdown();
    }

    // -------------------------------------------------------------------------
    // Flywheel readiness
    // -------------------------------------------------------------------------

    private void updateFlywheelReadiness(double errorRPM) {
        boolean nowInTol = Math.abs(errorRPM) <= ShooterConfig.VELOCITY_TOLERANCE_RPM && flywheelOn;
        if (nowInTol && !inTolerance) readyDwellTimer.reset();
        inTolerance   = nowInTol;
        flywheelReady = inTolerance
                        && readyDwellTimer.milliseconds() >= ShooterConfig.READY_DWELL_MS;
    }

    // -------------------------------------------------------------------------
    // Indexer state machine
    // -------------------------------------------------------------------------

    private void handleIndexerInput() {
        boolean canFire = !ShooterConfig.WAIT_FOR_FLYWHEEL_READY || flywheelReady;

        if (gamepad2.right_trigger > 0.2 && canFire) {
            if (indexerState == IndexerState.IDLE) {
                indexerState   = IndexerState.AUTO_FIRING;
                indexerPhaseOn = true;
                indexerTimer.reset();
            }
        } else if (indexerState == IndexerState.AUTO_FIRING) {
            indexerState = IndexerState.IDLE;
            indexerMotor.setPower(0);
        }

        if (gamepad2.x && xButtonTimer.milliseconds() > DEBOUNCE_MS && canFire) {
            if (indexerState == IndexerState.IDLE) {
                indexerState        = IndexerState.BURST_FIRING;
                burstBallsRemaining = ShooterConfig.BURST_COUNT;
                indexerPhaseOn      = true;
                indexerTimer.reset();
            }
            xButtonTimer.reset();
        }
    }

    private void tickIndexerStateMachine() {
        switch (indexerState) {
            case BURST_FIRING:
                if (indexerPhaseOn) {
                    indexerMotor.setPower(ShooterConfig.INDEXER_POWER);
                    if (indexerTimer.milliseconds() >= ShooterConfig.INDEXER_ON_TIME_MS) {
                        indexerPhaseOn = false;
                        indexerMotor.setPower(0);
                        indexerTimer.reset();
                        burstBallsRemaining--;
                        burstShotsFired++;
                    }
                } else {
                    if (indexerTimer.milliseconds() >= ShooterConfig.INDEXER_OFF_TIME_MS) {
                        if (burstBallsRemaining > 0) {
                            indexerPhaseOn = true;
                            indexerTimer.reset();
                        } else {
                            indexerState = IndexerState.IDLE;
                            indexerMotor.setPower(0);
                        }
                    }
                }
                break;

            case AUTO_FIRING:
                indexerMotor.setPower(ShooterConfig.INDEXER_POWER);
                long cycleDuration = ShooterConfig.INDEXER_ON_TIME_MS + ShooterConfig.INDEXER_OFF_TIME_MS;
                if (indexerTimer.milliseconds() > cycleDuration) {
                    autoShotCycles++;
                    indexerTimer.reset();
                }
                break;

            case IDLE:
            default:
                indexerMotor.setPower(0);
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Control handlers
    // -------------------------------------------------------------------------

    private void handleSpinToggle() {
        if (gamepad2.right_bumper) flywheelOn = true;
        if (gamepad2.left_bumper)  flywheelOn = false;
    }

    private void handleRPMAdjustment() {
        if (gamepad2.dpad_right && dpadRightTimer.milliseconds() > DEBOUNCE_MS) {
            targetRPM = Math.min(ShooterConfig.MOTOR_FREE_SPEED_RPM, targetRPM + ShooterConfig.RPM_STEP);
            dpadRightTimer.reset();
        }
        if (gamepad2.dpad_left && dpadLeftTimer.milliseconds() > DEBOUNCE_MS) {
            targetRPM = Math.max(0.0, targetRPM - ShooterConfig.RPM_STEP);
            dpadLeftTimer.reset();
        }
    }

    // -------------------------------------------------------------------------
    // Velocity ramp
    // -------------------------------------------------------------------------

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
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private DcMotor initIndexerMotor() {
        DcMotor motor = hardwareMap.get(DcMotor.class, ShooterConfig.INDEXER_MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void updateTelemetry(double actualRPM, double targetRPMNow,
                                  double errorRPM, double motorPower) {
        boolean withinTol = Math.abs(errorRPM) <= ShooterConfig.VELOCITY_TOLERANCE_RPM;

        telemetry.addLine("=== VERIFY TUNE (Step 4 of 4) ===");
        telemetry.addData("Session time", "%.1f s", sessionTimer.seconds());
        telemetry.addLine();

        // Spin-up time — the video's key metric (goal: < 0.25 s)
        telemetry.addLine("--- Spin-Up Performance ---");
        if (spinUpComplete) {
            telemetry.addData("Spin-up time", "%.0f ms  %s",
                    spinUpTimeMs,
                    spinUpTimeMs < 250 ? "✓ PASS (< 250 ms)" : "✗ Still slow — check KV");
        } else if (flywheelOn) {
            telemetry.addData("Spin-up time", "timing...");
        } else {
            telemetry.addData("Spin-up time", "--- (spin up to measure)");
        }
        telemetry.addLine();

        // Live velocity
        telemetry.addLine("--- Live Velocity ---");
        telemetry.addData("Target RPM",  "%.1f", targetRPMNow);
        telemetry.addData("Actual RPM",  "%.1f", actualRPM);
        telemetry.addData("Error RPM",   "%.1f  %s", errorRPM, withinTol ? "✓ PASS" : "✗ FAIL");
        telemetry.addData("Tolerance ±", "%.1f RPM", ShooterConfig.VELOCITY_TOLERANCE_RPM);
        telemetry.addData("Motor Power", "%.4f", motorPower);
        telemetry.addData("Ready",       flywheelReady ? "✓ YES" : "✗ spinning up");
        telemetry.addLine();

        // Indexer
        telemetry.addLine("--- Indexer ---");
        telemetry.addData("State",       indexerState.toString());
        telemetry.addData("Interlock",   ShooterConfig.WAIT_FOR_FLYWHEEL_READY ? "ON" : "OFF");
        telemetry.addLine();

        // Session statistics
        telemetry.addLine("--- Session Statistics ---");
        telemetry.addData("Burst shots fired", burstShotsFired);
        telemetry.addData("Auto-fire cycles",  autoShotCycles);
        telemetry.addData("Peak error (RPM)",  "%.1f  %s",
                peakErrorRPM,
                peakErrorRPM <= ShooterConfig.VELOCITY_TOLERANCE_RPM ? "✓ PASS" : "✗ FAIL");
        telemetry.addData("Ever out of tol?", everOutOfTol ? "✗ YES — review KP" : "✓ NO");
        telemetry.addLine();

        // Active coefficients (read-only in verify mode)
        telemetry.addLine("--- Active PVS Coefficients (ShooterConfig) ---");
        telemetry.addData("KS", "%.4f", ShooterConfig.KS_INITIAL);
        telemetry.addData("KV", "%.6f", ShooterConfig.KV_INITIAL);
        telemetry.addData("KP", "%.4f", ShooterConfig.KP_INITIAL);
        telemetry.addLine();

        // Verdict
        telemetry.addLine("--- Competition Readiness ---");
        boolean ready = !everOutOfTol
                        && spinUpComplete && spinUpTimeMs < 250
                        && (burstShotsFired > 0 || autoShotCycles > 0);
        boolean firingTested = burstShotsFired > 0 || autoShotCycles > 0;

        if (ready) {
            telemetry.addLine("✓✓✓  SYSTEM READY FOR COMPETITION  ✓✓✓");
        } else if (!firingTested) {
            telemetry.addLine("→ Fire burst (x) and auto-fire (rt) to complete validation.");
        } else if (!spinUpComplete || spinUpTimeMs >= 250) {
            telemetry.addLine("✗  Spin-up too slow — increase KV in ShooterConfig.");
        } else {
            telemetry.addLine("✗  Velocity left tolerance — increase KP or review KV/KS.");
        }

        telemetry.addLine();
        telemetry.addLine("rb=ON | lb=OFF | rt=auto | x=burst | dpad=RPM");
        telemetry.update();
    }

    // -------------------------------------------------------------------------
    // Shutdown
    // -------------------------------------------------------------------------

    private void shutdown() {
        if (flywheelMotor != null) flywheelMotor.setPower(0);
        if (indexerMotor  != null) indexerMotor.setPower(0);
    }
}
