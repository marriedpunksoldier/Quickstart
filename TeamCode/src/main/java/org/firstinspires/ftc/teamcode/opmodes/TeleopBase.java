package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Shooter.ShooterConfig;
import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.AllianceConfig;
import org.firstinspires.ftc.teamcode.config.DistanceTable;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveUtils;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.Locale;

/**
 * TeleopBase is the single implementation of teleop logic shared by both
 * alliances. Each alliance has a thin wrapper that calls super(Alliance.X).
 *
 * Replaces BlueTeleopv2 and RedTeleopv2 — which were ~1000 lines each with
 * ~990 lines of overlap.
 *
 * KEY BEHAVIORS
 * -------------
 *   - Driver 1 (gamepad1): drive only. Right stick X = rotate; left stick = translate.
 *   - Driver 2 (gamepad2):
 *       LB / LT       intake / outtake
 *       RB            spin shooter on (toggle); X = stop
 *       RT (rising)   FIRE a single ball (gated by readiness; ignores held trigger)
 *       D-pad up/down adjust manual power preset
 *       Y             toggle auto-distance mode (default ON)
 *       B             auto-move toward goal at AUTO_MOVE_TARGET_DISTANCE
 *
 * All bug fixes from prior reviews are baked in:
 *   - Discrete fire command via StopperSubsystem.tryFire
 *   - Auto-move uses targetHeading (faces the tag), not currentHeading
 *   - Turret has deadband + slew limit
 *   - Telemetry is decimated to every 4th loop
 *   - Single follower.update() call per loop
 *   - Auto-move has timeout escape
 *   - Voltage compensation in shooter
 */
public abstract class TeleopBase extends OpMode {

    // -------------------------------------------------------------------------
    // Auto-move tuning
    // -------------------------------------------------------------------------

    private static final double AUTO_MOVE_TARGET_DISTANCE = 60.0;  // inches
    private static final double AUTO_MOVE_TOLERANCE = 6.0;
    private static final double AUTO_MOVE_DRIVER_BREAKOUT = 0.15;
    private static final long AUTO_MOVE_TIMEOUT_MS = 2500;

    // -------------------------------------------------------------------------
    // Alliance + subsystems
    // -------------------------------------------------------------------------

    private final Alliance alliance;
    private final AllianceConfig allianceConfig;

    protected Follower follower;
    protected ShooterSubsystem shooter;
    protected LimelightSubsystem ll;
    protected TurretSubsystem turret;
    protected IntakeSubsystem intake;
    protected StopperSubsystem stopper;
    protected IndicatorSubsystem indicator;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    private boolean autoDistanceMode = true;        // Y toggles
    private int currentPresetIndex = 0;              // D-pad selects manual preset
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastY = false;
    private boolean lastRT = false;                  // for rising-edge fire trigger

    private boolean shooterOn = false;
    private boolean lastRB = false;
    private boolean lastX = false;

    private enum AutoMoveState { IDLE, MOVING }
    private AutoMoveState autoMoveState = AutoMoveState.IDLE;
    private long autoMoveStartMs = 0;
    private boolean lastB = false;
    private String autoMoveLastError = null;

    private int telemetryDivider = 0;
    private long lastLoopNanos = 0;

    // -------------------------------------------------------------------------

    protected TeleopBase(Alliance alliance) {
        this.alliance = alliance;
        this.allianceConfig = AllianceConfig.forAlliance(alliance);
    }

    // -------------------------------------------------------------------------
    // OpMode lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // ALWAYS set a starting pose. Don't conditionally check getPose()
        // first — depending on Pedro version, getPose() before the localizer
        // has reported can throw NPE itself. setPose() is safe to call
        // unconditionally and matches what AutoBase does.
        follower.setPose(getDefaultStartingPose());

        // Tell Pedro we're entering teleop drive mode. This must happen
        // BEFORE the first follower.update() call. The original
        // BlueTeleopv2 called this in init right after setPose, so we
        // match that pattern.
        follower.startTeleopDrive();

        shooter = new ShooterSubsystem(hardwareMap);
        ll = new LimelightSubsystem(hardwareMap, allianceConfig);
        turret = new TurretSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        stopper = new StopperSubsystem(hardwareMap);
        indicator = new IndicatorSubsystem(hardwareMap);

        telemetry.addData("Alliance", alliance);
        telemetry.addData("AprilTag ID", allianceConfig.aprilTagId);
        telemetry.addData("Limelight pipeline", allianceConfig.limelightPipeline);
        telemetry.addData("Limelight", ll.isConnected() ? "OK"
                : "FAILED: " + ll.getInitErrorMessage());
        telemetry.addData("Stopper hardware", stopper.isPresent() ? "OK" : "NOT FOUND");
        telemetry.update();
    }

    /**
     * Override to provide an alliance-appropriate starting pose. Default
     * is field origin, which is fine for driver practice. For matches,
     * autonomous sets the pose before teleop runs anyway.
     */
    protected Pose getDefaultStartingPose() {
        return new Pose(0, 0, 0);
    }

    @Override
    public void start() {
        lastLoopNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        // 0. Update the follower FIRST. This pulls the latest pose from
        //    the localizer into Pedro's internal cache so that any
        //    subsequent setTeleOpDrive() call has a valid heading to
        //    transform against. Without this, the very first loop iteration
        //    crashes inside VectorCalculator.setTeleOpMovementVectors
        //    when Pedro tries to read its cached heading.
        follower.update();

        // 1. Poll sensors once per loop
        ll.update();

        // 2. Input handling (note: intake is handled later — see step 6)
        handleAutoDistanceToggle();
        handleShooterToggle();
        handlePowerPresetSelection();
        handleAutoMoveTrigger();

        // 3. Update shooter target based on current mode
        updateShooterTarget();

        // 4. Aim turret (uses cached Limelight data)
        turret.aimAtTarget(ll.getTxDegrees(), ll.hasTarget());

        // 5. Determine if we're allowed to fire and handle the fire command
        boolean readyToFire = shooter.isReady() && turret.isLockedOn();
        boolean fireEdge = (gamepad2.right_trigger > 0.5) && !lastRT;
        stopper.tryFire(fireEdge, readyToFire);
        stopper.update();
        lastRT = (gamepad2.right_trigger > 0.5);

        // 6. Intake handling. Firing takes priority over driver intake input
        // because LB and RT shouldn't fight each other on the rear motor.
        // - If stopper is open: rear motor feeds ball through; front held off.
        // - Else: driver controls both motors via LB / LT.
        if (stopper.isOpen()) {
            intake.feedShooter();
        } else {
            handleIntake();
        }

        // 7. Driver feedback. LED tracks tag visibility, rumble fires on
        // full ready-to-shoot transition. Pass gamepad2 each call because
        // it isn't bound until after init() — must read it fresh each loop.
        indicator.update(ll.hasTarget(), readyToFire, gamepad2);

        // 8. Drive (skipped during auto-move)
        if (autoMoveState != AutoMoveState.MOVING) {
            handleDrive();
        } else {
            handleAutoMoveProgress();
        }

        // 9. Shooter control law applies last (after target is finalized this loop)
        shooter.update();

        // 10. Telemetry — decimated to ~12 Hz
        long now = System.nanoTime();
        double loopMs = (now - lastLoopNanos) / 1_000_000.0;
        lastLoopNanos = now;
        if (++telemetryDivider % 4 == 0) {
            displayTelemetry(loopMs, readyToFire);
        }
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stop();
        if (intake != null) intake.stop();
        if (stopper != null) stopper.forceClose();
        if (turret != null) turret.center();
        if (indicator != null) indicator.off(gamepad2);
    }

    // -------------------------------------------------------------------------
    // Input handlers (all rising-edge where appropriate)
    // -------------------------------------------------------------------------

    private void handleAutoDistanceToggle() {
        if (gamepad2.y && !lastY) {
            autoDistanceMode = !autoDistanceMode;
        }
        lastY = gamepad2.y;
    }

    private void handleShooterToggle() {
        if (gamepad2.right_bumper && !lastRB) shooterOn = true;
        if (gamepad2.x && !lastX) shooterOn = false;
        lastRB = gamepad2.right_bumper;
        lastX = gamepad2.x;
    }

    private void handlePowerPresetSelection() {
        if (gamepad2.dpad_up && !lastDpadUp) {
            currentPresetIndex = Math.min(allianceConfig.powerPresets.length - 1,
                    currentPresetIndex + 1);
        }
        if (gamepad2.dpad_down && !lastDpadDown) {
            currentPresetIndex = Math.max(0, currentPresetIndex - 1);
        }
        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;
    }

    private void handleIntake() {
        if (gamepad2.left_bumper) {
            intake.intake();
        } else if (gamepad2.left_trigger > 0.2) {
            intake.setPower(-gamepad2.left_trigger);  // proportional outtake
        } else {
            intake.stop();
        }
    }

    private void updateShooterTarget() {
        if (!shooterOn) {
            shooter.stop();
            return;
        }
        double power;
        if (autoDistanceMode && ll.hasTarget()) {
            power = ll.getInterpolatedPower(allianceConfig.powerPresets[currentPresetIndex]);
        } else {
            power = allianceConfig.powerPresets[currentPresetIndex];
        }
        shooter.setTargetByPower(power);
    }

    private void handleDrive() {
        double rawX = -gamepad1.left_stick_x;
        double rawY = -gamepad1.left_stick_y;
        double[] xy = DriveUtils.applyRadialDeadzone(rawX, rawY, DriveUtils.DEFAULT_DEADBAND);
        double strafe  = DriveUtils.squareCurve(xy[0]);
        double forward = DriveUtils.squareCurve(xy[1]);
        double rotate  = DriveUtils.squareCurve(DriveUtils.applyDeadzone(-gamepad1.right_stick_x));

        // Field-centric drive (the 4th 'true' argument) requires a valid
        // heading from the localizer. If the localizer hasn't reported a
        // pose yet, that call will NPE inside Pedro. Guard with a pose
        // check; fall back to robot-centric until the localizer is ready.
        boolean fieldCentric = false;
        try {
            Pose p = follower.getPose();
            fieldCentric = (p != null);
        } catch (Exception e) {
            fieldCentric = false;
        }

        follower.setTeleOpDrive(forward, strafe, rotate, fieldCentric);
    }

    // -------------------------------------------------------------------------
    // Auto-move — drives the robot toward the target AprilTag
    // -------------------------------------------------------------------------

    private void handleAutoMoveTrigger() {
        if (gamepad1.b && !lastB && autoMoveState == AutoMoveState.IDLE && ll.hasTarget()) {
            startAutoMove();
        }
        lastB = gamepad1.b;
    }

    private void startAutoMove() {
        try {
            double currentDistance = ll.getDistanceInches();
            double tx = ll.getTxDegrees();
            if (currentDistance <= AUTO_MOVE_TARGET_DISTANCE) return;

            Pose startPose = follower.getPose();
            if (startPose == null) {
                autoMoveLastError = "Auto-move skipped: no pose from localizer yet";
                return;
            }
            autoMoveLastError = null;
            double currentHeading = startPose.getHeading();

            // Heading-to-tag in field frame. The robot is offset from the tag by
            // -tx degrees (negative because positive tx means "right of camera").
            double targetHeading = currentHeading - Math.toRadians(tx);
            double delta = currentDistance - AUTO_MOVE_TARGET_DISTANCE;

            // Move along the heading vector toward the tag
            double targetX = startPose.getX() + delta * Math.cos(targetHeading);
            double targetY = startPose.getY() + delta * Math.sin(targetHeading);

            // CRITICAL FIX: end heading is the heading AT the tag, not the current
            // heading. The previous code drove to the spot but didn't rotate to
            // face the target, leaving the turret to make up the angular delta.
            Pose endPose = new Pose(targetX, targetY, targetHeading);

            PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(currentHeading, targetHeading)
                    .build();

            follower.followPath(path, true);
            autoMoveState = AutoMoveState.MOVING;
            autoMoveStartMs = System.currentTimeMillis();
        } catch (Exception e) {
            // Any unexpected error: log it, do not crash the OpMode.
            autoMoveLastError = "Auto-move error: " + e.getClass().getSimpleName()
                    + ": " + e.getMessage();
            autoMoveState = AutoMoveState.IDLE;
        }
    }

    private void handleAutoMoveProgress() {
        // Driver can interrupt with a small stick deflection
        double stickMag = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        boolean driverInterrupt = stickMag > AUTO_MOVE_DRIVER_BREAKOUT
                || Math.abs(gamepad1.right_stick_x) > AUTO_MOVE_DRIVER_BREAKOUT;
        boolean timedOut = (System.currentTimeMillis() - autoMoveStartMs) > AUTO_MOVE_TIMEOUT_MS;

        if (driverInterrupt || timedOut || !follower.isBusy()) {
            cancelAutoMove();
        }
    }

    private void cancelAutoMove() {
        // Re-enter teleop drive mode. Pedro requires this transition after
        // a path completes (otherwise setTeleOpDrive may not register).
        autoMoveState = AutoMoveState.IDLE;
        follower.startTeleopDrive();
        follower.setTeleOpDrive(0, 0, 0, false);  // robot-centric zero
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void displayTelemetry(double loopMs, boolean readyToFire) {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Loop ms", String.format(Locale.US, "%.1f", loopMs));
        telemetry.addLine();

        telemetry.addLine("--- Shooter ---");
        telemetry.addData("State", shooterOn ? "ON" : "off");
        telemetry.addData("Auto-distance mode", autoDistanceMode ? "ON (Y to disable)" : "off");
        telemetry.addData("Manual preset", currentPresetIndex + " ("
                + DistanceTable.DISTANCE_NAMES[currentPresetIndex] + ")");
        telemetry.addData("Target RPM", String.format(Locale.US, "%.0f", shooter.getTargetRPM()));
        telemetry.addData("Actual RPM", String.format(Locale.US, "%.0f", shooter.getActualRPM()));
        telemetry.addData("Motor out",  String.format(Locale.US, "%.2f", shooter.getMotorOutput()));
        telemetry.addData("Ready", shooter.isReady() ? "YES" : "spinning up...");
        if (shooter.isOverspeedTripped()) {
            telemetry.addLine(">>> OVERSPEED TRIP — restart shooter <<<");
        }
        if (shooter.isDirectionMismatched()) {
            telemetry.addLine(">>> MOTOR/ENCODER DIRECTION MISMATCH <<<");
        }
        telemetry.addLine();

        telemetry.addLine("--- Limelight ---");
        if (!ll.isConnected()) {
            telemetry.addLine("NOT CONNECTED: " + ll.getInitErrorMessage());
        } else {
            telemetry.addData("Pipeline",  ll.getConfiguredPipeline());
            telemetry.addData("Target ID", ll.getTargetTagId());
            telemetry.addData("Staleness", ll.getLastStalenessMs() < 0
                    ? "no result" : ll.getLastStalenessMs() + " ms");
            telemetry.addData("Visible tags", ll.getVisibleTagCount() == 0
                    ? "none" : ll.getVisibleTagCount() + " [" + ll.getVisibleTagIds() + "]");
            if (ll.hasTarget()) {
                telemetry.addData("Distance", String.format(Locale.US, "%.1f in", ll.getDistanceInches()));
                telemetry.addData("TX",       String.format(Locale.US, "%.1f deg", ll.getTxDegrees()));
            } else {
                telemetry.addData("State", "no target — " + ll.getLastSkipReason());
            }
        }
        telemetry.addLine();

        telemetry.addLine("--- Turret ---");
        telemetry.addData("Position", String.format(Locale.US, "%.3f", turret.getPosition()));
        telemetry.addData("Locked on", turret.isLockedOn() ? "YES" : "no");
        telemetry.addLine();

        telemetry.addData("READY TO FIRE", readyToFire ? "*** YES ***" : "no");
        telemetry.addData("Auto-move state", autoMoveState);
        if (autoMoveLastError != null) {
            telemetry.addLine(autoMoveLastError);
        }
        try {
            Pose currentPose = follower.getPose();
            if (currentPose != null) {
                telemetry.addData("Pose", String.format(Locale.US, "(%.1f, %.1f, %.1f°)",
                        currentPose.getX(), currentPose.getY(),
                        Math.toDegrees(currentPose.getHeading())));
            } else {
                telemetry.addLine("Pose: localizer not ready");
            }
        } catch (Exception e) {
            telemetry.addLine("Pose: error reading (" + e.getClass().getSimpleName() + ")");
        }
        telemetry.update();
    }
}
