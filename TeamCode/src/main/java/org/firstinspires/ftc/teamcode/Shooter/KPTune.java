package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * KPTune.java — Step 3 of 3 in the shooter tuning pipeline.
 *
 * PURPOSE
 * -------
 * Add proportional (KP) correction on top of the tuned KV + KS feedforward
 * to handle velocity drops when balls are fired through the flywheel.
 *
 * WHY THIS MATTERS (from the video)
 * ----------------------------------
 * "KP allows us to adapt to adjustments or drops in the system."
 * The feedforward (KV + KS) handles steady-state, but when a ball passes
 * through and briefly loads the flywheel, RPM drops.  KP provides the
 * extra corrective power burst to pull velocity back to target quickly.
 *
 * "We don't use integral or derivative on a flywheel — integral causes
 *  windup, derivative fights the inertia."  So KP is the ONLY feedback term.
 *
 * FULL CONTROL LAW
 * ----------------
 *   error          = targetRPM − actualRPM
 *   feedForward    = (KV × targetRPM) + KS
 *   pTerm          =  KP × error
 *   motorPower     =  feedForward + pTerm          (clamped to [0.0, 1.0])
 *
 * PRE-CONDITIONS
 * --------------
 *   KS and KV must already be tuned and set in ShooterConfig before running this.
 *
 * TUNING PROCEDURE
 * ----------------
 *   1. Press gamepad2 RIGHT_BUMPER to spin up the flywheel.
 *   2. Hold gamepad2 RIGHT_TRIGGER to feed balls continuously (auto-fire mode),
 *      OR press gamepad2 X to fire a burst.
 *   3. Watch "Actual RPM" on telemetry as balls fire.
 *   4. Increase KP (dpad_up) until:
 *        - The RPM dip after each ball is small (fast recovery)
 *        - BUT the RPM does NOT oscillate past the target after recovering.
 *      If you see oscillation (RPM swings above AND below target), KP is too high.
 *      Back off by 2–3 steps.
 *   5. Goal: velocity returns to within VELOCITY_TOLERANCE_RPM within ~0.3 s per shot.
 *   6. When satisfied, copy KP into ShooterConfig.KP_INITIAL, then run VerifyTune.java.
 *
 * CONTROLS (gamepad2 only)
 * -------------------------
 *   right_bumper              → spin flywheel ON
 *   left_bumper               → spin flywheel OFF
 *   dpad_up                   → increase KP by KP_STEP
 *   dpad_down                 → decrease KP by KP_STEP
 *   right_trigger (>0.2)      → auto-fire (continuous while held)
 *   x button                  → burst fire (BURST_COUNT balls)
 *   dpad_right                → increase target RPM by RPM_STEP
 *   dpad_left                 → decrease target RPM by RPM_STEP
 */
@TeleOp(name = "Shooter — 3. KP Tune  (Proportional Gain)", group = "Shooter Tuning")
public class KPTune extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotorEx flywheelMotor;
    private DcMotor   indexerMotor;

    // -------------------------------------------------------------------------
    // Live-tunable state
    // -------------------------------------------------------------------------
    private double kv        = ShooterConfig.KV_INITIAL;
    private double ks        = ShooterConfig.KS_INITIAL;
    private double kp        = ShooterConfig.KP_INITIAL;
    private double targetRPM = ShooterConfig.TARGET_RPM;
    private boolean flywheelOn = false;

    // Velocity ramp
    private double currentRampedRPM = 0.0;
    private final ElapsedTime rampTimer = new ElapsedTime();

    // Flywheel ready interlock
    private boolean inTolerance   = false;
    private boolean flywheelReady = false;
    private final ElapsedTime readyDwellTimer = new ElapsedTime();

    // Indexer state machine
    private enum IndexerState { IDLE, BURST_FIRING, AUTO_FIRING }
    private IndexerState indexerState   = IndexerState.IDLE;
    private int   burstBallsRemaining   = 0;
    private boolean indexerPhaseOn      = false;
    private final ElapsedTime indexerTimer = new ElapsedTime();

    // Shot counter (helps evaluate recovery time)
    private int totalShotsFired = 0;

    // Worst-case dip tracking
    private double peakErrorRPM = 0.0;

    // Debounce timers
    private final ElapsedTime dpadUpTimer    = new ElapsedTime();
    private final ElapsedTime dpadDownTimer  = new ElapsedTime();
    private final ElapsedTime dpadRightTimer = new ElapsedTime();
    private final ElapsedTime dpadLeftTimer  = new ElapsedTime();
    private final ElapsedTime xButtonTimer   = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final long DEBOUNCE_MS = 200;

    // -------------------------------------------------------------------------
    // Main OpMode
    // -------------------------------------------------------------------------
    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing KP Tune...");
        telemetry.update();

        flywheelMotor = initFlywheelMotor();
        indexerMotor  = initIndexerMotor();

        telemetry.addLine("✓ All motors initialized.");
        telemetry.addLine("Confirm KS and KV are set in ShooterConfig before running.");
        telemetry.addLine("Press PLAY to begin.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        rampTimer.reset();
        readyDwellTimer.reset();

        while (opModeIsActive()) {

            // --- Controls ---
            handleSpinToggle();
            handleKPAdjustment();
            handleRPMAdjustment();

            // --- Velocity ramp ---
            currentRampedRPM = rampRPM(currentRampedRPM, flywheelOn ? targetRPM : 0.0);

            // --- Read actual velocity ---
            // Switch to RUN_USING_ENCODER so getVelocity() returns reliable ticks/sec.
            // We still compute power ourselves (custom PVS) but need encoder feedback.
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double actualVelocityTPS = flywheelMotor.getVelocity();
            double actualRPM         = ticksPerSecondToRPM(Math.abs(actualVelocityTPS));
            double errorRPM          = currentRampedRPM - actualRPM;

            // --- PVS control law ---
            // feedForward = KV * targetRPM + KS
            // pTerm       = KP * error
            // motorPower  = feedForward + pTerm  (clamped)
            double feedForward  = (kv * currentRampedRPM) + ks;
            double pTerm        = kp * errorRPM;
            double motorPower   = Range.clip(feedForward + pTerm, 0.0, 1.0);

            flywheelMotor.setPower(flywheelOn ? motorPower : 0.0);

            // --- Track peak error under load ---
            if (flywheelOn && currentRampedRPM > 100) {
                peakErrorRPM = Math.max(peakErrorRPM, Math.abs(errorRPM));
            }

            // --- Flywheel ready interlock ---
            updateFlywheelReadiness(errorRPM);

            // --- Indexer ---
            handleIndexerInput();
            tickIndexerStateMachine();

            // --- Telemetry ---
            if (telemetryTimer.milliseconds() > ShooterConfig.TELEMETRY_UPDATE_MS) {
                updateTelemetry(actualRPM, currentRampedRPM, errorRPM, feedForward, pTerm, motorPower);
                telemetryTimer.reset();
            }
        }

        shutdown();
    }

    // -------------------------------------------------------------------------
    // Flywheel readiness
    // -------------------------------------------------------------------------

    private void updateFlywheelReadiness(double errorRPM) {
        boolean nowInTolerance = Math.abs(errorRPM) <= ShooterConfig.VELOCITY_TOLERANCE_RPM
                                 && flywheelOn;
        if (nowInTolerance && !inTolerance) readyDwellTimer.reset();
        inTolerance   = nowInTolerance;
        flywheelReady = inTolerance
                        && readyDwellTimer.milliseconds() >= ShooterConfig.READY_DWELL_MS;
    }

    // -------------------------------------------------------------------------
    // Indexer state machine
    // -------------------------------------------------------------------------

    private void handleIndexerInput() {
        boolean canFire = !ShooterConfig.WAIT_FOR_FLYWHEEL_READY || flywheelReady;

        // Auto-fire: right trigger held
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

        // Burst fire: x button
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
                        totalShotsFired++;
                        peakErrorRPM = 0.0; // reset after each shot so we see per-shot dip
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
                // Count shots during auto-fire (rough cycle count)
                if (indexerTimer.milliseconds() > ShooterConfig.INDEXER_ON_TIME_MS
                                                  + ShooterConfig.INDEXER_OFF_TIME_MS) {
                    totalShotsFired++;
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

    private void handleKPAdjustment() {
        if (gamepad2.dpad_up && dpadUpTimer.milliseconds() > DEBOUNCE_MS) {
            kp += ShooterConfig.KP_STEP;
            dpadUpTimer.reset();
        }
        if (gamepad2.dpad_down && dpadDownTimer.milliseconds() > DEBOUNCE_MS) {
            kp = Math.max(0.0, kp - ShooterConfig.KP_STEP);
            dpadDownTimer.reset();
        }
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

    private void updateTelemetry(double actualRPM, double targetRPMNow, double errorRPM,
                                  double feedForward, double pTerm, double motorPower) {
        boolean withinTol = Math.abs(errorRPM) <= ShooterConfig.VELOCITY_TOLERANCE_RPM;

        telemetry.addLine("=== KP TUNE — Proportional Gain ===");
        telemetry.addLine("(Step 3 of 3  |  Control law: power = KV×RPM + KS + KP×error)");
        telemetry.addLine();
        telemetry.addData("Flywheel",     flywheelOn ? "ON" : "OFF");
        telemetry.addData("Ready",        flywheelReady ? "✓ YES — indexer may fire" : "✗ spinning up...");
        telemetry.addLine();
        telemetry.addLine("--- Velocity ---");
        telemetry.addData("Target RPM",   "%.1f", targetRPMNow);
        telemetry.addData("Actual RPM",   "%.1f", actualRPM);
        telemetry.addData("Error RPM",    "%.1f  %s", errorRPM, withinTol ? "✓" : "✗");
        telemetry.addData("Peak Error",   "%.1f RPM  (resets per shot)", peakErrorRPM);
        telemetry.addLine();
        telemetry.addLine("--- PVS Decomposition ---");
        telemetry.addData("feedForward",  "%.4f  (KV×RPM + KS)", feedForward);
        telemetry.addData("pTerm",        "%.4f  (KP × error)",  pTerm);
        telemetry.addData("motorPower",   "%.4f  (total output)", motorPower);
        telemetry.addLine();
        telemetry.addLine("--- Coefficients ---");
        telemetry.addData("KS", "%.4f  (fixed)", ks);
        telemetry.addData("KV", "%.6f  (fixed)", kv);
        telemetry.addData("KP", "%.4f  ◄ TUNING NOW", kp);
        telemetry.addLine();
        telemetry.addLine("--- Indexer ---");
        telemetry.addData("State",        indexerState.toString());
        telemetry.addData("Total shots",  totalShotsFired);
        telemetry.addData("Interlock",    ShooterConfig.WAIT_FOR_FLYWHEEL_READY ? "ON" : "OFF");
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("rb=spin ON | lb=spin OFF");
        telemetry.addLine("dpad_up/down → KP ± " + ShooterConfig.KP_STEP);
        telemetry.addLine("rt=auto-fire | x=burst (" + ShooterConfig.BURST_COUNT + " balls)");
        telemetry.addLine("dpad_right/left → target RPM ± " + ShooterConfig.RPM_STEP);
        telemetry.addLine();
        telemetry.addLine("Goal: fast RPM recovery after each shot, no oscillation.");
        telemetry.addLine("Copy final KP → ShooterConfig.KP_INITIAL, then run VerifyTune.");
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
