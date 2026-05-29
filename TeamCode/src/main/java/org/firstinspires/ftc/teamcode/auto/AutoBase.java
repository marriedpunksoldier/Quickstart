package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Shooter.ShooterConfig;
import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.AllianceConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.Locale;

/**
 * AutoBase is the shared autonomous OpMode framework.
 *
 * Concrete subclasses (AutoLong, AutoShort, AutoMini, AutoQuick) override
 * {@link #buildPath()} to declare their routes in BLUE coordinates. The
 * helpers in this class automatically mirror to Red when needed.
 *
 * The shooting state machine — the part that's the same across all routes —
 * lives here. Subclasses only describe "drive to A, shoot 3, drive to B,
 * shoot 3, ..." as a high-level sequence; the framework handles the rest.
 *
 * KEY IMPROVEMENTS over the old per-route auto files
 * ---------------------------------------------------
 *   - Readiness uses dwell (no firing on transient zero-crossing).
 *   - "Ball fired" is detected by RPM dip+recovery, not a fixed timer.
 *   - No more MAX_SPINUP_TIMEOUT that fired regardless of readiness;
 *     the framework skips a shot if the flywheel never settles.
 *   - 29-second safety: at t=29s the robot stops all mechanisms and
 *     centers the turret no matter where it is in the sequence.
 *   - Pre-spin to the expected distance of the *next* shoot pose, so the
 *     flywheel is at the right RPM by the time the robot arrives.
 */
public abstract class AutoBase extends OpMode {

    // -------------------------------------------------------------------------
    // Timing / safety constants
    // -------------------------------------------------------------------------

    protected static final int BALLS_PER_POSITION = 3;
    /**
     * Maximum time to wait for the shooter to reach target speed before
     * giving up on a shot position. Set conservatively — a well-tuned
     * shooter spins up in <0.5s, but cold motors on first match of the
     * day, low battery, or refined RPM adjustments need more headroom.
     * If you see "flywheel never settled" warnings, your KSV tuning needs
     * work — don't just increase this number.
     */
    protected static final double MAX_SPINUP_SEC = 2.0;
    protected static final double RPM_DIP_THRESHOLD = 200;   // RPM drop = ball detected
    protected static final double SHOT_TIMEOUT_SEC = 1.5;    // give up on one shot
    protected static final double STOW_TIME_SEC = 29.0;      // start stowing
    protected static final double SAMPLE_SWEEP_POWER = 0.3;

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

    protected Timer opModeTimer;
    protected Timer actionTimer;

    // -------------------------------------------------------------------------
    // Shot state machine — local to each shooting block
    // -------------------------------------------------------------------------

    private int pathState = 0;
    private int ballsShot = 0;
    private int currentShotNumber = 0;
    private boolean shotInProgress = false;
    private double rpmBeforeShot = 0;
    private boolean stowed = false;

    // -------------------------------------------------------------------------

    protected AutoBase(Alliance alliance) {
        this.alliance = alliance;
        this.allianceConfig = AllianceConfig.forAlliance(alliance);
    }

    // -------------------------------------------------------------------------
    // Subclass contract
    // -------------------------------------------------------------------------

    /** Subclasses declare their starting pose in BLUE coordinates. */
    protected abstract Pose getStartPose();

    /**
     * Subclasses define the sequence of "drive to + shoot at" segments.
     * Called once during init. Each step should add a path via the helper
     * methods below.
     */
    protected abstract void buildPath();

    /**
     * The route sequence as a state-machine driver. Subclasses override
     * to define their own step ordering. Default: do nothing (override me).
     */
    protected abstract void onPathUpdate();

    // -------------------------------------------------------------------------
    // Public state for subclasses to read
    // -------------------------------------------------------------------------

    protected Alliance alliance()                { return alliance; }
    protected AllianceConfig allianceConfig()    { return allianceConfig; }

    /** Returns this pose mirrored to the active alliance side. */
    protected Pose pose(double x, double y, double headingDeg) {
        return alliance.mirror(new Pose(x, y, Math.toRadians(headingDeg)));
    }

    /** Pure heading mirror — for use with setLinearHeadingInterpolation. */
    protected double heading(double degreesBlue) {
        return alliance.mirrorHeading(Math.toRadians(degreesBlue));
    }

    // -------------------------------------------------------------------------
    // OpMode lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void init() {
        opModeTimer = new Timer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        Pose start = alliance.mirror(getStartPose());
        follower.setPose(start);

        shooter = new ShooterSubsystem(hardwareMap);
        ll      = new LimelightSubsystem(hardwareMap, allianceConfig);
        turret  = new TurretSubsystem(hardwareMap);
        intake  = new IntakeSubsystem(hardwareMap);
        stopper = new StopperSubsystem(hardwareMap);

        buildPath();

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start pose", start);
        telemetry.addData("AprilTag", allianceConfig.aprilTagId);
        telemetry.addData("Limelight", ll.isConnected() ? "OK"
                : "FAILED: " + ll.getInitErrorMessage());
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        actionTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        // Always-on safety: stow at the end-of-match
        if (!stowed && opModeTimer.getElapsedTimeSeconds() > STOW_TIME_SEC) {
            stowAll();
            stowed = true;
        }
        if (stowed) {
            shooter.update();
            stopper.update();
            return;
        }

        // Always poll Limelight and update shooter control law
        ll.update();
        shooter.update();
        stopper.update();
        turret.aimAtTarget(ll.getTxDegrees(), ll.hasTarget());

        // Subclass drives the state machine
        onPathUpdate();

        // Single follower update
        follower.update();

        // Light telemetry — autonomous doesn't need flashy displays
        addBasicTelemetry();
    }

    @Override
    public void stop() {
        stowAll();
    }

    private void stowAll() {
        if (shooter != null) shooter.stop();
        if (intake  != null) intake.stop();
        if (stopper != null) stopper.forceClose();
        if (turret  != null) turret.center();
    }

    // -------------------------------------------------------------------------
    // Building blocks for autonomous routes — used by onPathUpdate()
    // -------------------------------------------------------------------------

    /**
     * Build a path from currentPose to endPose (already mirrored if needed)
     * with linear heading interpolation. Convenience over raw pathBuilder.
     */
    protected PathChain straightPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    /**
     * Pre-spin the shooter to the expected distance at a shoot pose.
     * Pass the known distance from that pose to the goal (measured at the
     * field). Avoids the cold-spin-up wait at the shoot pose.
     */
    protected void preSpinForDistance(double expectedDistanceInches) {
        double power = (expectedDistanceInches > 0)
                ? org.firstinspires.ftc.teamcode.config.DistanceTable.interpolatePower(
                expectedDistanceInches, allianceConfig.powerPresets)
                : allianceConfig.powerPresets[0];
        shooter.setTargetByPower(power);
    }

    /** Refine target RPM using live Limelight data once at the shoot pose. */
    protected void refineRPMFromLimelight() {
        if (ll.hasTarget()) {
            double power = ll.getInterpolatedPower(allianceConfig.powerPresets[0]);
            shooter.setTargetByPower(power);
        }
    }

    // -------------------------------------------------------------------------
    // The shooting "burst" sub-state machine
    //
    // Subclasses call beginShootBurst() when they've arrived at a shoot pose,
    // then call updateShootBurst() each loop until it returns true (= done).
    // -------------------------------------------------------------------------

    protected void beginShootBurst(int shotNumber) {
        currentShotNumber = shotNumber;
        ballsShot = 0;
        shotInProgress = false;
        actionTimer.resetTimer();
    }

    /**
     * Run one tick of the shooting state machine.
     * Returns true once all BALLS_PER_POSITION balls have been processed
     * (whether successful or skipped due to flywheel never reaching speed).
     */
    protected boolean updateShootBurst() {
        if (ballsShot >= BALLS_PER_POSITION) return true;

        if (!shooter.isReady()) {
            // Either spinning up, or never reached target — give up after MAX_SPINUP_SEC
            if (actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_SEC) {
                telemetry.addLine("[!] Skipping shots at pos " + currentShotNumber
                        + " — flywheel never settled");
                ballsShot = BALLS_PER_POSITION;
                intake.stop();
                return true;
            }
            return false;
        }

        // In autonomous firing, run BOTH intake motors. There's no field-
        // side intake to fight, and balls picked up during the prior sweep
        // may be anywhere in the hopper — front needs to push them back so
        // rear can push them through to the shooter. This is different from
        // teleop, where the driver controls the front roller independently.
        intake.intake();

        // Ready: trigger the next shot if we're not already in one
        if (!shotInProgress) {
            rpmBeforeShot = shooter.getActualRPM();
            shotInProgress = true;
            // Open the stopper for one ball
            stopper.tryFire(true, true);  // (firePressed=true, readyToFire=true)
            actionTimer.resetTimer();
            return false;
        }

        // Shot in progress: wait for an RPM dip (ball fired) then recovery
        double dipSeen = rpmBeforeShot - shooter.getActualRPM();
        boolean recovered = shooter.isReady();  // back in tolerance with dwell

        if (dipSeen > RPM_DIP_THRESHOLD && recovered) {
            // Confirmed: a ball went through and the flywheel recovered
            ballsShot++;
            shotInProgress = false;
            actionTimer.resetTimer();
            if (ballsShot >= BALLS_PER_POSITION) {
                intake.stop();
                return true;
            }
            return false;
        }

        if (actionTimer.getElapsedTimeSeconds() > SHOT_TIMEOUT_SEC) {
            // Probable jam or missed feed — count it as attempted, move on
            telemetry.addLine("[!] Shot timeout at pos " + currentShotNumber);
            ballsShot++;
            shotInProgress = false;
            actionTimer.resetTimer();
            if (ballsShot >= BALLS_PER_POSITION) {
                intake.stop();
                return true;
            }
        }
        return ballsShot >= BALLS_PER_POSITION;
    }

    // -------------------------------------------------------------------------
    // Path-state convenience helpers for subclasses
    // -------------------------------------------------------------------------

    protected int pathState()              { return pathState; }
    protected void setPathState(int next)  { pathState = next; actionTimer.resetTimer(); }

    /** True after follower has arrived at the current path's endpoint. */
    protected boolean atDestination() {
        return !follower.isBusy();
    }

    /** Helper for sweeping (slow drive with intake on). */
    protected void beginSweep(PathChain path) {
        intake.intake();
        follower.setMaxPower(SAMPLE_SWEEP_POWER);
        follower.followPath(path, true);
    }

    /** Restore drive power after a sweep. */
    protected void endSweep() {
        follower.setMaxPower(1.0);
        intake.stop();
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void addBasicTelemetry() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Path state", pathState);
        telemetry.addData("OpMode time", String.format(Locale.US, "%.1f", opModeTimer.getElapsedTimeSeconds()));
        telemetry.addData("Action time", String.format(Locale.US, "%.2f", actionTimer.getElapsedTimeSeconds()));
        telemetry.addData("Shot #", currentShotNumber);
        telemetry.addData("Balls shot", ballsShot + "/" + BALLS_PER_POSITION);
        telemetry.addData("Shot in progress", shotInProgress);
        telemetry.addLine();
        telemetry.addData("Shooter target", String.format(Locale.US, "%.0f", shooter.getTargetRPM()));
        telemetry.addData("Shooter actual", String.format(Locale.US, "%.0f", shooter.getActualRPM()));
        telemetry.addData("Shooter error",  String.format(Locale.US, "%.0f", shooter.getError()));
        telemetry.addData("Shooter output", String.format(Locale.US, "%.3f", shooter.getMotorOutput()));
        telemetry.addData("Shooter ready",  shooter.isReady());
        telemetry.addLine();
        if (ll.hasTarget()) {
            telemetry.addData("LL distance", String.format(Locale.US, "%.1f in", ll.getDistanceInches()));
            telemetry.addData("LL TX",       String.format(Locale.US, "%.1f deg", ll.getTxDegrees()));
        } else {
            telemetry.addData("LL", "no tag (" + ll.getLastSkipReason() + ")");
        }
        telemetry.update();
    }
}
