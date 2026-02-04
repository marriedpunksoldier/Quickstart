package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.List;

@Autonomous(name = "RedAutov3", group = "Autonomous")
public class RedAutov3 extends OpMode {
    // Pedro Pathing
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Hardware
    private DcMotor frontIntake;
    private DcMotorEx shooter;
    private Servo turretGear;
    private Servo pusherServo;

    // Limelight
    private Limelight3A limelight;
    private boolean limelightConnected = false;

    // ═══════════════════════════════════════════════════════════════════
    // DISTANCE-POWER LOOKUP TABLE (from RedTeleop)
    // ═══════════════════════════════════════════════════════════════════
    // Distances in inches
    private static final double[] DISTANCE_PRESETS = {24.0, 36.0, 48.0, 60.0, 72.0, 84.0};
    // Corresponding power levels (0.0 - 1.0)
    private static final double[] POWER_PRESETS =    {0.71, 0.77, 0.83, 0.89, 0.95, 1.0};
    // Distance preset names for display
    private static final String[] DISTANCE_NAMES = {"2 ft", "3 ft", "4 ft", "5 ft", "6 ft", "7 ft"};

    // Fallback power if no AprilTag detected
    private static final double FALLBACK_SHOOTER_POWER = 0.60;

    // ═══════════════════════════════════════════════════════════════════
    // FLYWHEEL SHOOTER CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════

    // Motor specifications (goBILDA 5203 series 1:1 ratio)
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double TICKS_PER_SECOND_AT_MAX_RPM = (MOTOR_MAX_RPM / 60.0) * MOTOR_TICKS_PER_REV;

    // PIDF coefficients (from testing)
    private static final double SHOOTER_KP = 10.0;
    private static final double SHOOTER_KI = 0.100;
    private static final double SHOOTER_KD = 0.1;
    private static final double SHOOTER_KF = 12.0;

    // Velocity verification settings
    private static final double VELOCITY_TOLERANCE_PERCENT = 2.0;
    private static final double MAX_SPINUP_TIME = 2.0;

    // ═══════════════════════════════════════════════════════════════════
    // APRILTAG AUTOTRACKING CONFIGURATION (from RedTeleop)
    // ═══════════════════════════════════════════════════════════════════

    // Turret servo configuration
    private static final double TURRET_CENTER = 0.5;
    private static final double TURRET_MIN = 0.0;
    private static final double TURRET_MAX = 1.0;
    // goBILDA Super Speed servo: 90 degrees across full range (0.0 to 1.0)
    private static final double TURRET_DEGREES_PER_SERVO_UNIT = 90.0;

    // Target AprilTag ID (-1 = use any/closest tag)
    private static final int TARGET_APRILTAG_ID = 24;

    // Limelight settings
    private static final int LIMELIGHT_PIPELINE = 6;  // Preset for AprilTag detection

    // ═══════════════════════════════════════════════════════════════════

    // Constants for servo positions and motor powers
    private static final double INTAKE_POWER = 1.0;
    private static final double PUSHER_RETRACTED = 0.2;
    private static final double PUSHER_EXTENDED = 0.5;

    // Timing constants (in seconds)
    private static final double PUSH_TIME = 0.2;     // Time to wait while pushing a ball
    private static final double RETRACT_TIME = 0.5;  // Time to wait after retracting for balls to move through intake

    // Multi-shot configuration
    private static final int BALLS_PER_POSITION = 3;  // Shoot 3 balls at each position

    // Shooter state tracking
    private double currentTargetVelocity = 0;
    private boolean shooterVelocityReached = false;
    private int currentShotNumber = 0;
    private int ballsShot = 0;  // Track balls shot at current position

    // AprilTag tracking state
    private boolean limelightHasTarget = false;
    private double limelightDistance = 0;
    private double limelightTx = 0;
    private double autoPowerLevel = 0;
    private double turretPosition = TURRET_CENTER;

    // ==================== Poses ====================
    // Start position
    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));

    // Shooting positions
    private final Pose shoot1Pose = new Pose(83.88599670510709, 96.3163097199341, Math.toRadians(45));
    private final Pose shoot2Pose = new Pose(83.88599670510709, 96.3163097199341, Math.toRadians(45));
    private final Pose shoot3Pose = new Pose(83.88599670510709, 96.3163097199341, Math.toRadians(45));
    private final Pose shoot4Pose = new Pose(83.88599670510709, 96.3163097199341, Math.toRadians(45));

    // Waypoints for sample sweeping
    private final Pose waypoint1Pose = new Pose(102.72158154859966, 35.58484349258651, Math.toRadians(0));
    private final Pose waypoint2Pose = new Pose(129.2915980230642, 35.34761120263588, Math.toRadians(0));
    private final Pose waypoint3Pose = new Pose(102.72158154859966, 35.58484349258651, Math.toRadians(0));

    private final Pose waypoint4Pose = new Pose(101, 59.30807248764413, Math.toRadians(0));
    private final Pose waypoint5Pose = new Pose(129, 59.54530477759474, Math.toRadians(0));
    private final Pose waypoint6Pose = new Pose(101, 59.30807248764413, Math.toRadians(0));

    private final Pose waypoint7Pose = new Pose(102.66128500823724, 84.21746293245471, Math.toRadians(0));
    private final Pose waypoint8Pose = new Pose(128, 83.2685337726524, Math.toRadians(0));

    // ==================== Path Chains ====================
    private PathChain path1_toShoot1;
    private PathChain path2_toWaypoint1;
    private PathChain path3_toWaypoint2;
    private PathChain path4_toWaypoint3;
    private PathChain path5_toShoot2;
    private PathChain path6_toWaypoint4;
    private PathChain path7_toWaypoint5;
    private PathChain path8_toWaypoint6;
    private PathChain path9_toShoot3;
    private PathChain path10_toWaypoint7;
    private PathChain path11_toWaypoint8;
    private PathChain path12_toShoot4;

    public void buildPaths() {
        // Path 1: Start -> Shoot 1
        path1_toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        // Path 2: Shoot 1 -> Waypoint 1
        path2_toWaypoint1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, waypoint1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 3: Waypoint 1 -> Waypoint 2
        path3_toWaypoint2 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint1Pose, waypoint2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 4: Waypoint 2 -> Waypoint 3
        path4_toWaypoint3 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint2Pose, waypoint3Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 5: Waypoint 3 -> Shoot 2
        path5_toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint3Pose, shoot2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        // Path 6: Shoot 2 -> Waypoint 4
        path6_toWaypoint4 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, waypoint4Pose))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 7: Waypoint 4 -> Waypoint 5
        path7_toWaypoint5 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint4Pose, waypoint5Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 8: Waypoint 5 -> Waypoint 6
        path8_toWaypoint6 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint5Pose, waypoint6Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        // Path 9: Waypoint 6 -> Shoot 3
        path9_toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint6Pose, shoot3Pose))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        // Path 10: Shoot 3 -> Waypoint 7
        path10_toWaypoint7 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, waypoint7Pose))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 11: Waypoint 7 -> Waypoint 8
        path11_toWaypoint8 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint7Pose, waypoint8Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 12: Waypoint 8 -> Shoot 4 (End)
        path12_toShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint8Pose, shoot4Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }

    // ==================== Hardware Control Methods ====================

    private void startIntake() {
        frontIntake.setPower(INTAKE_POWER);
    }

    private void stopIntake() {
        frontIntake.setPower(0);
    }

    /**
     * Start shooter with velocity control.
     * Uses Limelight distance-based power if AprilTag visible,
     * otherwise falls back to preset power level.
     */
    private void startShooter(int shotNumber) {
        currentShotNumber = shotNumber;
        shooterVelocityReached = false;

        // Use auto-calculated power if AprilTag visible, otherwise fallback
        double powerLevel;
        if (limelightHasTarget && autoPowerLevel > 0) {
            powerLevel = autoPowerLevel;
        } else {
            powerLevel = FALLBACK_SHOOTER_POWER;
        }

        // Calculate target velocity
        currentTargetVelocity = powerLevel * TICKS_PER_SECOND_AT_MAX_RPM;

        // Set velocity (PIDF controller handles the rest)
        shooter.setVelocity(currentTargetVelocity);
    }

    /**
     * Check if shooter has reached target velocity
     */
    private void updateShooterVelocityStatus() {
        if (currentTargetVelocity == 0) {
            shooterVelocityReached = false;
            return;
        }

        double currentVelocity = shooter.getVelocity();
        double error = Math.abs(currentTargetVelocity - currentVelocity);
        double errorPercent = (error / currentTargetVelocity) * 100.0;
        shooterVelocityReached = (errorPercent < VELOCITY_TOLERANCE_PERCENT);
    }

    private void stopShooter() {
        shooter.setPower(0);
        currentTargetVelocity = 0;
        shooterVelocityReached = false;
    }

    private void pushSample() {
        pusherServo.setPosition(PUSHER_EXTENDED);
    }

    private void retractPusher() {
        pusherServo.setPosition(PUSHER_RETRACTED);
    }

    private void stopAllMechanisms() {
        stopIntake();
        stopShooter();
        retractPusher();
    }

    // ==================== Limelight AprilTag Tracking ====================
    // NOTE: Limelight is used ONLY for AprilTag tracking and shooter power
    // Localization is handled exclusively by GoBilda Pinpoint odometry

    /**
     * Updates AprilTag tracking data from Limelight.
     * Gets distance for power calculation and tx for turret aiming.
     * Does NOT update robot pose - that comes only from Pinpoint.
     */
    private void updateLimelightTracking() {
        limelightHasTarget = false;
        limelightDistance = 0;
        limelightTx = 0;
        autoPowerLevel = 0;

        if (!limelightConnected) {
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            return;
        }

        // Find the target AprilTag (or use closest if TARGET_APRILTAG_ID is -1)
        LLResultTypes.FiducialResult targetTag = null;
        double closestDistance = Double.MAX_VALUE;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            // Get distance from robot to tag using robot-space pose
            Pose3D tagPose = fr.getRobotPoseTargetSpace();
            if (tagPose == null) {
                continue;
            }

            // Calculate 3D distance (convert meters to inches)
            double x = tagPose.getPosition().x * 39.3701;
            double y = tagPose.getPosition().y * 39.3701;
            double z = tagPose.getPosition().z * 39.3701;
            double distance = Math.sqrt(x*x + y*y + z*z);

            // Check if this is our target tag or the closest one
            if (TARGET_APRILTAG_ID == 24) {
                // Use closest tag
                if (distance < closestDistance) {
                    closestDistance = distance;
                    targetTag = fr;
                    limelightDistance = distance;
                }
            } else if (fr.getFiducialId() == TARGET_APRILTAG_ID) {
                targetTag = fr;
                limelightDistance = distance;
                break;
            }
        }

        if (targetTag != null && limelightDistance > 0) {
            limelightHasTarget = true;
            autoPowerLevel = interpolatePower(limelightDistance);
            // Get horizontal offset for turret tracking
            limelightTx = targetTag.getTargetXDegrees();
        }
    }

    /**
     * Interpolates power level based on distance using the lookup table.
     * Uses linear interpolation between the two closest distance presets.
     */
    private double interpolatePower(double distance) {
        // Clamp distance to table range
        if (distance <= DISTANCE_PRESETS[0]) {
            return POWER_PRESETS[0];
        }
        if (distance >= DISTANCE_PRESETS[DISTANCE_PRESETS.length - 1]) {
            return POWER_PRESETS[POWER_PRESETS.length - 1];
        }

        // Find the two closest presets and interpolate
        for (int i = 0; i < DISTANCE_PRESETS.length - 1; i++) {
            if (distance >= DISTANCE_PRESETS[i] && distance <= DISTANCE_PRESETS[i + 1]) {
                double t = (distance - DISTANCE_PRESETS[i]) /
                        (DISTANCE_PRESETS[i + 1] - DISTANCE_PRESETS[i]);
                return POWER_PRESETS[i] + t * (POWER_PRESETS[i + 1] - POWER_PRESETS[i]);
            }
        }

        return FALLBACK_SHOOTER_POWER;
    }

    /**
     * Aims turret using AprilTag autotracking.
     * Uses tx (horizontal offset) to calculate servo position.
     * Falls back to center if no target visible.
     */
    private void aimTurretWithTracking() {
        if (limelightHasTarget) {
            // Calculate target servo position from tx offset
            double servoOffset = limelightTx / TURRET_DEGREES_PER_SERVO_UNIT;
            double targetPosition = TURRET_CENTER - servoOffset;

            // Clamp to servo range
            turretPosition = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetPosition));
        } else {
            // No target - use center position
            turretPosition = TURRET_CENTER;
        }

        turretGear.setPosition(turretPosition);
    }

    /**
     * Returns telemetry data about AprilTag tracking.
     */
    private void addLimelightTelemetry() {
        telemetry.addLine("─── LIMELIGHT TRACKING ───");

        if (!limelightConnected) {
            telemetry.addData("Status", "Not Connected");
            return;
        }

        telemetry.addData("AprilTag", limelightHasTarget ? "DETECTED" : "NOT FOUND");

        if (limelightHasTarget) {
            telemetry.addData("Distance", "%.1f in", limelightDistance);
            telemetry.addData("TX (offset)", "%.1f deg", limelightTx);
            telemetry.addData("Auto Power", "%.0f%%", autoPowerLevel * 100);
            telemetry.addData("Turret Pos", "%.3f", turretPosition);
        }

        // Show detected tag IDs
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                StringBuilder tagIds = new StringBuilder();
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    tagIds.append(fr.getFiducialId()).append(" ");
                }
                telemetry.addData("Tag IDs", tagIds.toString());
            }
        }
    }

    /**
     * Add detailed shooter telemetry
     */
    private void addShooterTelemetry() {
        double currentVelocity = shooter.getVelocity();
        double currentRPM = (currentVelocity / MOTOR_TICKS_PER_REV) * 60.0;
        double targetRPM = (currentTargetVelocity / MOTOR_TICKS_PER_REV) * 60.0;

        telemetry.addLine("─── SHOOTER ───");
        telemetry.addData("Position", currentShotNumber);
        telemetry.addData("Balls Shot", "%d / %d", ballsShot, BALLS_PER_POSITION);

        if (limelightHasTarget) {
            telemetry.addData("Power Source", "AUTO (%.1f in)", limelightDistance);
        } else {
            telemetry.addData("Power Source", "FALLBACK");
        }

        telemetry.addData("Power Level", "%.0f%%",
                (limelightHasTarget ? autoPowerLevel : FALLBACK_SHOOTER_POWER) * 100);
        telemetry.addData("Current RPM", "%.0f", currentRPM);
        telemetry.addData("Target RPM", "%.0f", targetRPM);

        if (currentTargetVelocity > 0) {
            double error = Math.abs(currentTargetVelocity - currentVelocity);
            double errorPercent = (error / currentTargetVelocity) * 100.0;
            telemetry.addData("Velocity Error", "%.1f%%", errorPercent);
            telemetry.addData("Ready", shooterVelocityReached ? "YES" : "NO");
        }
    }

    // ==================== State Machine ====================
    // Each shooting position fires 3 balls with intake running throughout

    public void autonomousPathUpdate() {
        // Update shooter velocity status every loop
        updateShooterVelocityStatus();

        switch (pathState) {
            // ===== Drive to Shoot Position 1 =====
            case 0:
                follower.followPath(path1_toShoot1, true);
                setPathState(1);
                break;

            // ===== Score at Position 1 (3 balls) =====
            case 1:
                if (!follower.isBusy()) {
                    updateLimelightTracking();
                    startShooter(1);  // Start shooter first
                    aimTurretWithTracking();
                    ballsShot = 0;
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for velocity to stabilize OR timeout, then start intake and shoot
                if (shooterVelocityReached ||
                        actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_TIME) {
                    startIntake();  // Start intake after shooter is at speed
                    pushSample();
                    setPathState(3);
                }
                break;

            case 3:
                // Push complete, retract
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    retractPusher();
                    ballsShot++;
                    setPathState(4);
                }
                break;

            case 4:
                // Check if more balls to shoot at this position
                if (actionTimer.getElapsedTimeSeconds() > RETRACT_TIME) {
                    if (ballsShot < BALLS_PER_POSITION) {
                        // Shoot another ball
                        pushSample();
                        setPathState(3);  // Go back to push completion check
                    } else {
                        // Done shooting, move to next waypoint
                        stopShooter();
                        follower.followPath(path2_toWaypoint1, true);
                        setPathState(5);
                    }
                }
                break;

            // ===== Sweep 1: Waypoint 1 -> 2 -> 3 -> Shoot 2 =====
            case 5:
                if (!follower.isBusy()) {
                    // Intake already running from shooting
                    follower.followPath(path3_toWaypoint2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(path4_toWaypoint3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(path5_toShoot2, true);
                    setPathState(8);
                }
                break;

            // ===== Score at Position 2 (3 balls) =====
            case 8:
                if (!follower.isBusy()) {
                    updateLimelightTracking();
                    startShooter(2);  // Start shooter first
                    aimTurretWithTracking();
                    ballsShot = 0;
                    setPathState(9);
                }
                break;

            case 9:
                if (shooterVelocityReached ||
                        actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_TIME) {
                    startIntake();  // Start intake after shooter is at speed
                    pushSample();
                    setPathState(10);
                }
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    retractPusher();
                    ballsShot++;
                    setPathState(11);
                }
                break;

            case 11:
                if (actionTimer.getElapsedTimeSeconds() > RETRACT_TIME) {
                    if (ballsShot < BALLS_PER_POSITION) {
                        pushSample();
                        setPathState(10);
                    } else {
                        stopShooter();
                        follower.followPath(path6_toWaypoint4, true);
                        setPathState(12);
                    }
                }
                break;

            // ===== Sweep 2: Waypoint 4 -> 5 -> 6 -> Shoot 3 =====
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(path7_toWaypoint5, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(path8_toWaypoint6, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(path9_toShoot3, true);
                    setPathState(15);
                }
                break;

            // ===== Score at Position 3 (3 balls) =====
            case 15:
                if (!follower.isBusy()) {
                    updateLimelightTracking();
                    startShooter(3);  // Start shooter first
                    aimTurretWithTracking();
                    ballsShot = 0;
                    setPathState(16);
                }
                break;

            case 16:
                if (shooterVelocityReached ||
                        actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_TIME) {
                    startIntake();  // Start intake after shooter is at speed
                    pushSample();
                    setPathState(17);
                }
                break;

            case 17:
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    retractPusher();
                    ballsShot++;
                    setPathState(18);
                }
                break;

            case 18:
                if (actionTimer.getElapsedTimeSeconds() > RETRACT_TIME) {
                    if (ballsShot < BALLS_PER_POSITION) {
                        pushSample();
                        setPathState(17);
                    } else {
                        stopShooter();
                        follower.followPath(path10_toWaypoint7, true);
                        setPathState(19);
                    }
                }
                break;

            // ===== Sweep 3: Waypoint 7 -> 8 -> Shoot 4 =====
            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(path11_toWaypoint8, true);
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    follower.followPath(path12_toShoot4, true);
                    setPathState(21);
                }
                break;

            // ===== Score at Position 4 (3 balls) =====
            case 21:
                if (!follower.isBusy()) {
                    updateLimelightTracking();
                    startShooter(4);  // Start shooter first
                    aimTurretWithTracking();
                    ballsShot = 0;
                    setPathState(22);
                }
                break;

            case 22:
                if (shooterVelocityReached ||
                        actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_TIME) {
                    startIntake();  // Start intake after shooter is at speed
                    pushSample();
                    setPathState(23);
                }
                break;

            case 23:
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    retractPusher();
                    ballsShot++;
                    setPathState(24);
                }
                break;

            case 24:
                if (actionTimer.getElapsedTimeSeconds() > RETRACT_TIME) {
                    if (ballsShot < BALLS_PER_POSITION) {
                        pushSample();
                        setPathState(23);
                    } else {
                        // All done!
                        stopAllMechanisms();
                        setPathState(-1);
                    }
                }
                break;

            default:
                // Done
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        // Initialize Pedro Pathing follower (Pinpoint localization ONLY)
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        // Initialize intake motors
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize shooter motor with velocity control
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
                SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF
        );
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Initialize servos
        turretGear = hardwareMap.get(Servo.class, "turretGear");
        pusherServo = hardwareMap.get(Servo.class, "pusherServo");
        turretGear.setPosition(TURRET_CENTER);
        pusherServo.setPosition(PUSHER_RETRACTED);
        turretPosition = TURRET_CENTER;

        // Initialize Limelight (for tracking ONLY, not localization)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();
            limelightConnected = true;
        } catch (Exception e) {
            limelightConnected = false;
        }

        // Build paths
        buildPaths();

        // Display initialization info
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("RedAutov3 - AUTO DISTANCE");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("Localization: Pinpoint ONLY");
        telemetry.addLine("Limelight: Tracking & Power");
        telemetry.addData("Balls per Position", BALLS_PER_POSITION);
        telemetry.addLine("Shooter spins up before intake");
        telemetry.addLine("────────────────────────────────");
        telemetry.addLine("Distance-Power Table:");
        for (int i = 0; i < DISTANCE_PRESETS.length; i++) {
            telemetry.addData("  " + DISTANCE_NAMES[i],
                    String.format("%.0f%%", POWER_PRESETS[i] * 100));
        }
        telemetry.addLine("────────────────────────────────");
        telemetry.addData("Limelight", limelightConnected ? "Connected" : "NOT FOUND");
        telemetry.addData("Fallback Power", "%.0f%%", FALLBACK_SHOOTER_POWER * 100);
        telemetry.addData("Battery", "%.2fV",
                hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addLine("────────────────────────────────");
        telemetry.addLine("Ready to Start");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        // Update Pinpoint odometry (ONLY source of localization)
        follower.update();

        // Update AprilTag tracking (for turret aim and shooter power ONLY)
        updateLimelightTracking();

        // Run state machine
        autonomousPathUpdate();

        // Telemetry - Robot State
        telemetry.addData("Path State", pathState);
        telemetry.addData("Opmode Time", "%.1f s", opmodeTimer.getElapsedTimeSeconds());

        // Telemetry - Pinpoint Position (ONLY localization source)
        telemetry.addLine("─── PINPOINT POSITION ───");
        telemetry.addData("X", "%.1f in", follower.getPose().getX());
        telemetry.addData("Y", "%.1f in", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(follower.getPose().getHeading()));

        // Telemetry - Shooter
        addShooterTelemetry();

        // Telemetry - Limelight Tracking
        addLimelightTelemetry();

        // Telemetry - Mechanisms
        telemetry.addLine("─── MECHANISMS ───");
        telemetry.addData("Intake", frontIntake.getPower() > 0 ? "ON" : "OFF");
        telemetry.addData("Pusher", pusherServo.getPosition() > 0.3 ? "EXTENDED" : "RETRACTED");

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelightConnected) {
            limelight.stop();
        }
        stopAllMechanisms();
    }
}
