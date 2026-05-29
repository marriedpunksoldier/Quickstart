package org.firstinspires.ftc.teamcode.Testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.AllianceConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveUtils;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * PowerCalibration — calibration tool to dial in shooter power at known distances.
 *
 * WORKFLOW
 * --------
 *   1. Position the robot so the Limelight sees the goal AprilTag.
 *      The "Distance" telemetry value shows what the camera measures.
 *   2. Move to a specific distance (e.g. 36 inches). Confirm steady reading.
 *   3. Spin up the shooter with RB. Adjust power with dpad up/down.
 *      Hold L3 (left stick button) for coarse 0.05 increments,
 *      otherwise fine 0.005 increments.
 *   4. Pull RT to fire one ball. Watch where it lands.
 *   5. Tweak power, fire again. Repeat until the shot lands cleanly.
 *   6. Press A to SAVE the current (distance, power) as a calibration point.
 *   7. Reposition the robot to the next distance and repeat.
 *   8. When done, the "Calibration Table" telemetry shows every saved
 *      point as a Java array — copy/paste those values into AllianceConfig.
 *
 * SAVED POINTS PERSIST IN MEMORY ONLY (not flash). Don't restart the
 * OpMode before you've captured what you need. Take a photo of the
 * telemetry showing the calibration table when you're done.
 *
 * USES THE REAL ARCHITECTURE
 * --------------------------
 * This OpMode uses the same ShooterSubsystem, LimelightSubsystem, etc.
 * that match teleop and autonomous use. Power values you dial in here
 * are guaranteed to behave identically in matches because there's only
 * one implementation of the control law.
 *
 * CONTROLS (gamepad2)
 *   right_bumper    spin shooter ON
 *   X               spin shooter OFF
 *   dpad_up         power +0.005 (or +0.05 with L3 held)
 *   dpad_down       power -0.005 (or -0.05 with L3 held)
 *   A               save current (distance, power) point
 *   B               delete the last saved point
 *   start           clear ALL saved points
 *   right_trigger   fire one ball (edge-triggered, gated on ready)
 *   left_bumper     intake both motors (collect balls)
 *   left_trigger    proportional outtake (clear jams)
 *
 * CONTROLS (gamepad1)
 *   left/right stick   drive
 */
@TeleOp(name = "Power Calibration Tool", group = "Test")
public class PowerCalibration extends OpMode {

    // -----------------------------------------------------------------
    // Tunable behavior
    // -----------------------------------------------------------------

    /** Which alliance config to use (controls tag ID + pipeline). */
    private static final Alliance ALLIANCE = Alliance.BLUE;

    private static final double POWER_STEP_FINE   = 0.005;
    private static final double POWER_STEP_COARSE = 0.050;
    private static final double POWER_MIN = 0.10;
    private static final double POWER_MAX = 1.00;

    private static final double STARTING_POWER = 0.55;

    // -----------------------------------------------------------------
    // State
    // -----------------------------------------------------------------

    private Follower follower;
    private ShooterSubsystem shooter;
    private LimelightSubsystem ll;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private StopperSubsystem stopper;

    private final AllianceConfig allianceConfig = AllianceConfig.forAlliance(ALLIANCE);

    private double currentPower = STARTING_POWER;
    private boolean shooterOn = false;

    /** Saved (distance, power) calibration points. */
    private final List<CalibrationPoint> calibrationPoints = new ArrayList<>();

    // Edge-tracking
    private boolean lastRT = false;
    private boolean lastRB = false;
    private boolean lastX = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastStart = false;

    private final ElapsedTime dpadUpTimer = new ElapsedTime();
    private final ElapsedTime dpadDownTimer = new ElapsedTime();
    private static final long DPAD_DEBOUNCE_MS = 150;

    private long lastShotMs = 0;
    private int shotsFired = 0;

    // -----------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();

        shooter = new ShooterSubsystem(hardwareMap);
        ll      = new LimelightSubsystem(hardwareMap, allianceConfig);
        turret  = new TurretSubsystem(hardwareMap);
        intake  = new IntakeSubsystem(hardwareMap);
        stopper = new StopperSubsystem(hardwareMap);

        telemetry.addLine("=== POWER CALIBRATION TOOL ===");
        telemetry.addData("Alliance", ALLIANCE);
        telemetry.addData("Target tag ID", allianceConfig.aprilTagId);
        telemetry.addData("Pipeline", allianceConfig.limelightPipeline);
        telemetry.addData("Limelight", ll.isConnected() ? "OK"
                : "FAILED: " + ll.getInitErrorMessage());
        telemetry.addData("Starting power", currentPower);
        telemetry.addLine();
        telemetry.addLine("Press PLAY to begin.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 0. Update follower first so Pedro's pose cache is populated
        follower.update();

        // 1. Poll sensors
        ll.update();

        // 2. Handle inputs
        handleShooterToggle();
        handlePowerAdjustment();
        handleCalibrationPointInput();
        handleIntake();

        // 3. Update shooter target
        if (shooterOn) {
            shooter.setTargetByPower(currentPower);
        } else {
            shooter.stop();
        }

        // 4. Aim turret at tag (read-only — calibration doesn't depend on
        //    turret lock, but the visualisation is helpful)
        turret.aimAtTarget(ll.getTxDegrees(), ll.hasTarget());

        // 5. Handle firing — gated on shooter ready + turret locked + tag
        boolean readyToFire = shooter.isReady() && turret.isLockedOn() && ll.hasTarget();
        boolean fireEdge = (gamepad2.right_trigger > 0.5) && !lastRT;
        boolean fired = stopper.tryFire(fireEdge, readyToFire);
        stopper.update();
        if (fired) {
            shotsFired++;
            lastShotMs = System.currentTimeMillis();
        }
        lastRT = (gamepad2.right_trigger > 0.5);

        // 6. Feed shooter while stopper is open
        if (stopper.isOpen()) {
            intake.feedShooter();
        }

        // 7. Drive
        handleDrive();

        // 8. Update shooter control law
        shooter.update();

        // 9. Telemetry
        displayTelemetry(readyToFire);
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stop();
        if (intake != null) intake.stop();
        if (stopper != null) stopper.forceClose();
        if (turret != null) turret.center();
    }

    // -----------------------------------------------------------------
    // Input handlers
    // -----------------------------------------------------------------

    private void handleShooterToggle() {
        boolean rb = gamepad2.right_bumper;
        boolean x = gamepad2.x;
        if (rb && !lastRB) shooterOn = true;
        if (x && !lastX)   shooterOn = false;
        lastRB = rb;
        lastX = x;
    }

    private void handlePowerAdjustment() {
        double step = gamepad2.left_stick_button ? POWER_STEP_COARSE : POWER_STEP_FINE;

        if (gamepad2.dpad_up && dpadUpTimer.milliseconds() > DPAD_DEBOUNCE_MS) {
            currentPower = Math.min(POWER_MAX, currentPower + step);
            dpadUpTimer.reset();
        }
        if (gamepad2.dpad_down && dpadDownTimer.milliseconds() > DPAD_DEBOUNCE_MS) {
            currentPower = Math.max(POWER_MIN, currentPower - step);
            dpadDownTimer.reset();
        }
    }

    private void handleCalibrationPointInput() {
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;
        boolean start = gamepad2.start;

        // A — save current (distance, power) as a calibration point
        if (a && !lastA && ll.hasTarget()) {
            calibrationPoints.add(new CalibrationPoint(ll.getDistanceInches(), currentPower));
            // Keep them sorted by distance for easy review
            Collections.sort(calibrationPoints);
            // Quick gamepad buzz to confirm
            gamepad2.rumble(0.6, 0.6, 150);
        }

        // B — remove the most recently added (by insertion order) point.
        // Since we sort on insert, "most recent" means the one closest to
        // current distance — that's usually what you want anyway.
        if (b && !lastB && !calibrationPoints.isEmpty() && ll.hasTarget()) {
            double currentDist = ll.getDistanceInches();
            int bestIdx = 0;
            double bestDelta = Double.MAX_VALUE;
            for (int i = 0; i < calibrationPoints.size(); i++) {
                double d = Math.abs(calibrationPoints.get(i).distanceInches - currentDist);
                if (d < bestDelta) {
                    bestDelta = d;
                    bestIdx = i;
                }
            }
            calibrationPoints.remove(bestIdx);
        }

        // START — wipe all points (with intentional friction: must hold)
        if (start && !lastStart) {
            calibrationPoints.clear();
        }

        lastA = a;
        lastB = b;
        lastStart = start;
    }

    private void handleIntake() {
        if (stopper.isOpen()) return;   // firing takes priority

        if (gamepad2.left_bumper) {
            intake.intake();
        } else if (gamepad2.left_trigger > 0.2) {
            intake.setPower(-gamepad2.left_trigger);
        } else {
            intake.stop();
        }
    }

    private void handleDrive() {
        double rawX = -gamepad1.left_stick_x;
        double rawY = -gamepad1.left_stick_y;
        double[] xy = DriveUtils.applyRadialDeadzone(rawX, rawY, DriveUtils.DEFAULT_DEADBAND);
        double strafe  = DriveUtils.squareCurve(xy[0]);
        double forward = DriveUtils.squareCurve(xy[1]);
        double rotate  = DriveUtils.squareCurve(DriveUtils.applyDeadzone(-gamepad1.right_stick_x));

        // Robot-centric drive is more predictable for calibration — driver
        // doesn't need to worry about field orientation when positioning.
        follower.setTeleOpDrive(forward, strafe, rotate, false);
    }

    // -----------------------------------------------------------------
    // Telemetry
    // -----------------------------------------------------------------

    private void displayTelemetry(boolean readyToFire) {
        telemetry.addLine("=== POWER CALIBRATION ===");
        telemetry.addLine();

        // Big readable current values
        telemetry.addLine("--- Right Now ---");
        if (ll.hasTarget()) {
            telemetry.addData("Distance", String.format(Locale.US, "%.1f in", ll.getDistanceInches()));
        } else {
            telemetry.addData("Distance", "NO TAG (" + ll.getLastSkipReason() + ")");
        }
        telemetry.addData("Power",    String.format(Locale.US, "%.3f  (%.0f%%)",
                currentPower, currentPower * 100));
        telemetry.addData("Step",     gamepad2.left_stick_button
                ? String.format(Locale.US, "%.3f (coarse, hold L3)", POWER_STEP_COARSE)
                : String.format(Locale.US, "%.3f (fine, hold L3 for coarse)", POWER_STEP_FINE));
        telemetry.addLine();

        // Shooter status
        telemetry.addLine("--- Shooter ---");
        telemetry.addData("State", shooterOn ? "ON" : "off (RB to start, X to stop)");
        if (shooterOn) {
            telemetry.addData("Target RPM", String.format(Locale.US, "%.0f", shooter.getTargetRPM()));
            telemetry.addData("Actual RPM", String.format(Locale.US, "%.0f", shooter.getActualRPM()));
            telemetry.addData("Error",      String.format(Locale.US, "%.0f", shooter.getError()));
            telemetry.addData("Motor out",  String.format(Locale.US, "%.3f", shooter.getMotorOutput()));
            telemetry.addData("Ready",      shooter.isReady() ? "YES" : "spinning up...");
        }
        if (shooter.isOverspeedTripped()) {
            telemetry.addLine(">>> OVERSPEED TRIP — restart with RB <<<");
        }
        telemetry.addLine();

        // Limelight
        telemetry.addLine("--- Limelight ---");
        telemetry.addData("Tag visible", ll.hasTarget() ? "YES (tag " + allianceConfig.aprilTagId + ")"
                : "no");
        if (ll.getVisibleTagCount() > 0) {
            telemetry.addData("All visible tags", ll.getVisibleTagIds());
        }
        if (ll.hasTarget()) {
            telemetry.addData("TX", String.format(Locale.US, "%.1f deg", ll.getTxDegrees()));
        }
        telemetry.addData("Turret locked", turret.isLockedOn() ? "YES" : "no");
        telemetry.addLine();

        // Fire status
        telemetry.addLine("--- Fire ---");
        telemetry.addData("Ready to fire", readyToFire ? "*** YES — pull RT ***" : "no");
        telemetry.addData("Shots fired this session", shotsFired);
        if (lastShotMs > 0) {
            long since = (System.currentTimeMillis() - lastShotMs);
            if (since < 2000) {
                telemetry.addData("Last shot",
                        String.format(Locale.US, "%.1fs ago at power %.3f, dist %.1f in",
                                since / 1000.0, currentPower,
                                ll.hasTarget() ? ll.getDistanceInches() : -1));
            }
        }
        telemetry.addLine();

        // Calibration points
        telemetry.addLine("--- Calibration Points ---");
        if (calibrationPoints.isEmpty()) {
            telemetry.addLine("(none yet — press A to save current dist+power)");
        } else {
            telemetry.addData("Total saved", calibrationPoints.size());
            telemetry.addLine();
            telemetry.addLine("Distance (in) -> Power");
            for (CalibrationPoint p : calibrationPoints) {
                telemetry.addLine(String.format(Locale.US, "  %6.1f  ->  %.3f  (%.0f%%)",
                        p.distanceInches, p.power, p.power * 100));
            }
            telemetry.addLine();
            telemetry.addLine("Copy/paste into AllianceConfig:");
            telemetry.addLine("DISTANCE_PRESETS:");
            telemetry.addLine("  " + formatDistanceArray());
            telemetry.addLine("powerPresets:");
            telemetry.addLine("  " + formatPowerArray());
        }
        telemetry.addLine();

        // Controls hint
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("RB/X: shooter on/off");
        telemetry.addLine("dpad up/down: power (L3 = coarse)");
        telemetry.addLine("A: save point   B: delete nearest");
        telemetry.addLine("RT: fire one ball");
        telemetry.addLine("START: clear all points");

        telemetry.update();
    }

    private String formatDistanceArray() {
        StringBuilder sb = new StringBuilder("{");
        for (int i = 0; i < calibrationPoints.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(String.format(Locale.US, "%.1f", calibrationPoints.get(i).distanceInches));
        }
        sb.append("}");
        return sb.toString();
    }

    private String formatPowerArray() {
        StringBuilder sb = new StringBuilder("{");
        for (int i = 0; i < calibrationPoints.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(String.format(Locale.US, "%.3f", calibrationPoints.get(i).power));
        }
        sb.append("}");
        return sb.toString();
    }

    // -----------------------------------------------------------------
    // Helper class
    // -----------------------------------------------------------------

    private static final class CalibrationPoint implements Comparable<CalibrationPoint> {
        final double distanceInches;
        final double power;

        CalibrationPoint(double distanceInches, double power) {
            this.distanceInches = distanceInches;
            this.power = power;
        }

        @Override
        public int compareTo(CalibrationPoint other) {
            return Double.compare(this.distanceInches, other.distanceInches);
        }
    }
}
