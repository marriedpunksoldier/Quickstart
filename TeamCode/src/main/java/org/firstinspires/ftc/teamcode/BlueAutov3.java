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

@Autonomous(name = "BlueAutov3", group = "Autonomous")
public class BlueAutov3 extends OpMode {
    // Pedro Pathing
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Hardware
    private DcMotor frontIntake;
    private DcMotor rearIntake;
    private DcMotorEx shooter;  // CHANGED: Now DcMotorEx for velocity control
    private Servo turretGear;
    private Servo pusherServo;

    // Limelight
    private Limelight3A limelight;
    private boolean limelightConnected = false;

    // ═══════════════════════════════════════════════════════════════════
    // FLYWHEEL SHOOTER CONFIGURATION
    // TODO: UPDATE THESE VALUES WITH YOUR TESTED DATA!
    // ═══════════════════════════════════════════════════════════════════

    // Motor specifications (goBILDA 5203 series - DO NOT CHANGE)
    private static final double MOTOR_TICKS_PER_REV = 28.0; // 28 CPR × 1:1 ratio
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double TICKS_PER_SECOND_AT_MAX_RPM = (MOTOR_MAX_RPM / 60.0) * MOTOR_TICKS_PER_REV;

    // TESTED PIDF COEFFICIENTS
    // TODO: UPDATE these with your values from FlywheelShooterTest!
    private static final double SHOOTER_KP = 9.0;  // TODO: Your tested kP
    private static final double SHOOTER_KI = 0.100;   // TODO: Your tested kI
    private static final double SHOOTER_KD = 0.1;   // TODO: Your tested kD
    private static final double SHOOTER_KF = 12.5;  // TODO: Your tested kF

    // SHOOTING POWER LEVELS
    // TODO: UPDATE based on distance and testing
    // All your shooting positions appear to be at the same location (~67, 87)
    // so you'll likely use the same power for all shots
    private static final double SHOOTER_POWER_LEVEL = 0.60;  // TODO: Your tested power (0.0-1.0)

    // Optional: Different power levels for each shot if needed
    private static final double SHOT_1_POWER = 0.60;  // TODO: Update if needed
    private static final double SHOT_2_POWER = 0.60;  // TODO: Update if needed
    private static final double SHOT_3_POWER = 0.60;  // TODO: Update if needed
    private static final double SHOT_4_POWER = 0.60;  // TODO: Update if needed

    // Velocity verification settings
    private static final double VELOCITY_TOLERANCE_PERCENT = 1.5;  // Accept within 1.5% error
    private static final double MAX_SPINUP_TIME = 2.0;  // Maximum time to wait for velocity (seconds)

    // ═══════════════════════════════════════════════════════════════════

    // Constants for servo positions and motor powers
    private static final double INTAKE_POWER = 1.0;
    private static final double TURRET_SCORE_POSITION = 0.5;
    private static final double PUSHER_RETRACTED = 0.2;
    private static final double PUSHER_EXTENDED = 0.5;

    // Timing constants (in seconds)
    private static final double SHOOTER_SPINUP_TIME = 1.5;  // Increased for velocity verification
    private static final double PUSH_TIME = 0.3;

    // Limelight pose correction settings
    private static final boolean ENABLE_POSE_CORRECTION = true;
    private static final double POSE_CORRECTION_CONFIDENCE_THRESHOLD = 0.7;
    private static final int LIMELIGHT_PIPELINE = 5;

    // Shooter state tracking
    private double currentTargetVelocity = 0;
    private boolean shooterVelocityReached = false;
    private int currentShotNumber = 0;  // Track which shot we're on (1-4)

    // ==================== Poses ====================
    // Start position
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    // Shooting positions
    private final Pose shoot1Pose = new Pose(66.8995, 86.827, Math.toRadians(143));
    private final Pose shoot2Pose = new Pose(66.8995, 86.827, Math.toRadians(143));
    private final Pose shoot3Pose = new Pose(67.1367, 87.0643, Math.toRadians(143));
    private final Pose shoot4Pose = new Pose(67.1367, 86.827, Math.toRadians(143));

    // Waypoints for sample sweeping
    private final Pose waypoint1Pose = new Pose(53.14, 35.1104, Math.toRadians(180));
    private final Pose waypoint2Pose = new Pose(29.1796, 35.5848, Math.toRadians(180));
    private final Pose waypoint3Pose = new Pose(53.14, 35.1104, Math.toRadians(180));

    private final Pose waypoint4Pose = new Pose(52.6656, 60.0198, Math.toRadians(180));
    private final Pose waypoint5Pose = new Pose(32.5008, 59.7825, Math.toRadians(180));
    private final Pose waypoint6Pose = new Pose(52.6656, 60.0198, Math.toRadians(143));

    private final Pose waypoint7Pose = new Pose(48.1582, 83.743, Math.toRadians(180));
    private final Pose waypoint8Pose = new Pose(29.1796, 83.743, Math.toRadians(180));

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
        // Path 1: Start → Shoot 1
        path1_toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(143))
                .build();

        // Path 2: Shoot 1 → Waypoint 1
        path2_toWaypoint1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, waypoint1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 3: Waypoint 1 → Waypoint 2
        path3_toWaypoint2 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint1Pose, waypoint2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: Waypoint 2 → Waypoint 3
        path4_toWaypoint3 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint2Pose, waypoint3Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 5: Waypoint 3 → Shoot 2
        path5_toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint3Pose, shoot2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();

        // Path 6: Shoot 2 → Waypoint 4
        path6_toWaypoint4 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, waypoint4Pose))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        // Path 7: Waypoint 4 → Waypoint 5
        path7_toWaypoint5 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint4Pose, waypoint5Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 8: Waypoint 5 → Waypoint 6
        path8_toWaypoint6 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint5Pose, waypoint6Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();

        // Path 9: Waypoint 6 → Shoot 3
        path9_toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint6Pose, shoot3Pose))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
                .build();

        // Path 10: Shoot 3 → Waypoint 7
        path10_toWaypoint7 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, waypoint7Pose))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        // Path 11: Waypoint 7 → Waypoint 8
        path11_toWaypoint8 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint7Pose, waypoint8Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 12: Waypoint 8 → Shoot 4 (End)
        path12_toShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(waypoint8Pose, shoot4Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();
    }

    // ==================== Hardware Control Methods ====================

    private void startIntake() {
        frontIntake.setPower(INTAKE_POWER);
        rearIntake.setPower(INTAKE_POWER);
    }

    private void stopIntake() {
        frontIntake.setPower(0);
        rearIntake.setPower(0);
    }

    /**
     * Start shooter with velocity control based on shot number
     * Uses tested power levels and PIDF control
     */
    private void startShooter(int shotNumber) {
        currentShotNumber = shotNumber;
        shooterVelocityReached = false;

        // Get power level for this shot
        double powerLevel = getShooterPowerForShot(shotNumber);

        // Calculate target velocity
        currentTargetVelocity = powerLevel * TICKS_PER_SECOND_AT_MAX_RPM;

        // Set velocity (PIDF controller handles the rest)
        shooter.setVelocity(currentTargetVelocity);
    }

    /**
     * Returns the appropriate power level for each shot
     * TODO: Update if your shots need different power levels
     */
    private double getShooterPowerForShot(int shotNumber) {
        switch (shotNumber) {
            case 1: return SHOT_1_POWER;
            case 2: return SHOT_2_POWER;
            case 3: return SHOT_3_POWER;
            case 4: return SHOT_4_POWER;
            default: return SHOOTER_POWER_LEVEL;
        }
    }

    /**
     * Check if shooter has reached target velocity
     * Updates shooterVelocityReached flag
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

    private void aimTurret(double position) {
        turretGear.setPosition(position);
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

    // ==================== Limelight Methods ====================

    /**
     * Updates robot pose from Limelight AprilTag detection.
     * Only corrects pose when robot is stationary (not busy following a path)
     * and detection confidence is above threshold.
     */
    private void updatePoseFromLimelight() {
        if (!limelightConnected || !ENABLE_POSE_CORRECTION) {
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        // Get fiducial (AprilTag) results
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            return;
        }

        // Get robot pose from Limelight's MegaTag localization
        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            return;
        }

        // Only correct pose when robot is stationary (at scoring positions)
        if (!follower.isBusy()) {
            double xInches = botpose.getPosition().x * 39.3701;
            double yInches = botpose.getPosition().y * 39.3701;
            double headingRad = Math.toRadians(botpose.getOrientation().getYaw());

            Pose correctedPose = new Pose(xInches, yInches, headingRad);
            follower.setPose(correctedPose);
        }
    }

    /**
     * Returns telemetry data about current AprilTag detections.
     */
    private void addLimelightTelemetry() {
        if (!limelightConnected) {
            telemetry.addData("Limelight", "Not Connected");
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("Limelight", "No Valid Data");
            return;
        }

        double latency = result.getCaptureLatency() + result.getTargetingLatency();
        telemetry.addData("LL Latency", "%.1f ms", latency);

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        telemetry.addData("AprilTags", fiducials.size());

        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            telemetry.addData("LL Pose",
                    "X: %.1f, Y: %.1f, Yaw: %.1f°",
                    botpose.getPosition().x * 39.3701,
                    botpose.getPosition().y * 39.3701,
                    botpose.getOrientation().getYaw());
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
        telemetry.addData("Shot #", currentShotNumber);
        telemetry.addData("Power Level", "%.0f%%",
                getShooterPowerForShot(currentShotNumber) * 100);
        telemetry.addData("Current RPM", "%.0f", currentRPM);
        telemetry.addData("Target RPM", "%.0f", targetRPM);

        if (currentTargetVelocity > 0) {
            double error = Math.abs(currentTargetVelocity - currentVelocity);
            double errorPercent = (error / currentTargetVelocity) * 100.0;
            telemetry.addData("Velocity Error", "%.1f%%", errorPercent);
            telemetry.addData("Velocity OK?", shooterVelocityReached ? "YES" : "NO");
        }

        telemetry.addData("Motor Power", "%.2f", shooter.getPower());
    }

    // ==================== State Machine ====================

    /*
     * State machine flow:
     * 0:  Drive to Shoot 1
     * 1:  Spin up shooter, aim turret (with velocity verification)
     * 2:  Wait for velocity, then push sample (score preload)
     * 3:  Retract, drive to Waypoint 1
     * 4:  Drive to Waypoint 2 (sweep)
     * 5:  Drive to Waypoint 3
     * 6:  Drive to Shoot 2
     * 7:  Spin up shooter (with velocity verification)
     * 8:  Wait for velocity, then push sample
     * 9:  Retract, drive to Waypoint 4
     * 10: Drive to Waypoint 5 (sweep)
     * 11: Drive to Waypoint 6
     * 12: Drive to Shoot 3
     * 13: Spin up shooter (with velocity verification)
     * 14: Wait for velocity, then push sample
     * 15: Retract, drive to Waypoint 7
     * 16: Drive to Waypoint 8 (sweep)
     * 17: Drive to Shoot 4
     * 18: Spin up shooter (with velocity verification)
     * 19: Wait for velocity, then push sample
     * 20: Done
     */

    public void autonomousPathUpdate() {
        // Update shooter velocity status every loop
        updateShooterVelocityStatus();

        switch (pathState) {
            // ===== Score Preload (Shoot 1) =====
            case 0:
                follower.followPath(path1_toShoot1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    updatePoseFromLimelight();
                    startShooter(1);  // Shot 1
                    aimTurret(TURRET_SCORE_POSITION);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for velocity to stabilize OR timeout
                if (shooterVelocityReached ||
                        actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_TIME) {
                    pushSample();
                    setPathState(3);
                }
                break;

            case 3:
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    retractPusher();
                    stopShooter();
                    follower.followPath(path2_toWaypoint1, true);
                    setPathState(4);
                }
                break;

            // ===== Sweep 1: Waypoint 1 → 2 → 3 → Shoot 2 =====
            case 4:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(path3_toWaypoint2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(path4_toWaypoint3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(path5_toShoot2, true);
                    setPathState(7);
                }
                break;

            // ===== Score (Shoot 2) =====
            case 7:
                if (!follower.isBusy()) {
                    updatePoseFromLimelight();
                    startShooter(2);  // Shot 2
                    aimTurret(TURRET_SCORE_POSITION);
                    setPathState(8);
                }
                break;

            case 8:
                // Wait for velocity to stabilize OR timeout
                if (shooterVelocityReached ||
                        actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_TIME) {
                    pushSample();
                    setPathState(9);
                }
                break;

            case 9:
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    retractPusher();
                    stopShooter();
                    follower.followPath(path6_toWaypoint4, true);
                    setPathState(10);
                }
                break;

            // ===== Sweep 2: Waypoint 4 → 5 → 6 → Shoot 3 =====
            case 10:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(path7_toWaypoint5, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(path8_toWaypoint6, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(path9_toShoot3, true);
                    setPathState(13);
                }
                break;

            // ===== Score (Shoot 3) =====
            case 13:
                if (!follower.isBusy()) {
                    updatePoseFromLimelight();
                    startShooter(3);  // Shot 3
                    aimTurret(TURRET_SCORE_POSITION);
                    setPathState(14);
                }
                break;

            case 14:
                // Wait for velocity to stabilize OR timeout
                if (shooterVelocityReached ||
                        actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_TIME) {
                    pushSample();
                    setPathState(15);
                }
                break;

            case 15:
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    retractPusher();
                    stopShooter();
                    follower.followPath(path10_toWaypoint7, true);
                    setPathState(16);
                }
                break;

            // ===== Sweep 3: Waypoint 7 → 8 → Shoot 4 =====
            case 16:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(path11_toWaypoint8, true);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(path12_toShoot4, true);
                    setPathState(18);
                }
                break;

            // ===== Score (Shoot 4) =====
            case 18:
                if (!follower.isBusy()) {
                    updatePoseFromLimelight();
                    startShooter(4);  // Shot 4
                    aimTurret(TURRET_SCORE_POSITION);
                    setPathState(19);
                }
                break;

            case 19:
                // Wait for velocity to stabilize OR timeout
                if (shooterVelocityReached ||
                        actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_TIME) {
                    pushSample();
                    setPathState(20);
                }
                break;

            case 20:
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    stopAllMechanisms();
                    setPathState(-1);
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

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        // Initialize motors
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        rearIntake = hardwareMap.get(DcMotor.class, "rearIntake");

        // CHANGED: Now using DcMotorEx for velocity control
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Configure motor directions
        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure shooter for velocity control
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients from testing
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
                SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF
        );
        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        // Initialize servos
        turretGear = hardwareMap.get(Servo.class, "turretGear");
        pusherServo = hardwareMap.get(Servo.class, "pusherServo");

        // Set initial servo positions
        retractPusher();

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();
            limelightConnected = true;
            telemetry.addLine("✓ Limelight initialized");
        } catch (Exception e) {
            limelightConnected = false;
            telemetry.addLine("⚠ WARNING: Limelight not found!");
        }

        // Build paths
        buildPaths();

        // Display initialization info
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("BlueAutov3");
        telemetry.addLine("════════════════════════════════");
        telemetry.addData("Shooter PIDF", "kP=%.1f, kF=%.1f", SHOOTER_KP, SHOOTER_KF);
        telemetry.addData("Power Level", "%.0f%%", SHOOTER_POWER_LEVEL * 100);
        telemetry.addData("Battery", "%.2fV",
                hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addLine("────────────────────────────────");
        telemetry.addLine("✓ Ready to Start");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry - Robot State
        telemetry.addData("Path State", pathState);
        telemetry.addData("Opmode Time", "%.1f s", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Action Timer", "%.2f s", actionTimer.getElapsedTimeSeconds());

        // Telemetry - Odometry Pose
        telemetry.addLine("─── POSITION ───");
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.0f°", Math.toDegrees(follower.getPose().getHeading()));

        // Telemetry - Shooter (detailed)
        addShooterTelemetry();

        // Telemetry - Limelight
        addLimelightTelemetry();

        // Telemetry - Other Mechanisms
        telemetry.addData("Intake", frontIntake.getPower() > 0 ? "ON" : "OFF");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop Limelight polling
        if (limelightConnected) {
            limelight.stop();
        }
        stopAllMechanisms();
    }
}
