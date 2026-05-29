package org.firstinspires.ftc.teamcode.archive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Shooter.ShooterConfig;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "RedQuick", group = "Autonomous")
@Disabled
public class RedQuick extends OpMode{
    // Pedro Pathing
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Hardware
    private DcMotor frontIntake;
    private DcMotor rearIntake;
    private DcMotorEx shooter;
    private Servo turretGear;

    // Limelight
    private Limelight3A limelight;
    private boolean limelightConnected = false;

    // Indicator LED
    private Servo indicator;
    private static final double INDICATOR_GREEN = 0.500;
    private static final double INDICATOR_OFF   = 0.28;

    // ═══════════════════════════════════════════════════════════════════
    // DISTANCE-POWER LOOKUP TABLE
    // ═══════════════════════════════════════════════════════════════════
    // Distances in inches
    private static final double[] DISTANCE_PRESETS = {24.0, 36.0, 48.0, 60.0, 72.0, 84.0, 120.0, 132.0};
    // Corresponding power levels (0.0 - 1.0)
    private static final double[] POWER_PRESETS =   {0.50, 0.50, 0.50, 0.55, 0.58, 0.60, 0.72, 0.75};
    // Fallback power if limelight has no target
    private static final double FALLBACK_SHOOTER_POWER = 0.55;

    // ═══════════════════════════════════════════════════════════════════
    // FLYWHEEL SHOOTER CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════

    // KSV shooter control constants sourced from ShooterConfig
    private static final double MAX_SPINUP_TIME = 2.0;

    // ═══════════════════════════════════════════════════════════════════
    // APRILTAG AUTOTRACKING CONFIGURATION
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
    private static final int LIMELIGHT_PIPELINE = 2;  // Configure this pipeline for AprilTag detection with appropriate camera settings

    // ═══════════════════════════════════════════════════════════════════

    // Constants for servo positions and motor powers
    private static final double INTAKE_POWER = 1.0;
    // Timing constants (in seconds)
    private static final double PUSH_TIME = 0.2;     // Time to wait while pushing a ball
    private static final double RETRACT_TIME = 0.5;  // Time to wait after retracting for balls to move through intake

    // Multi-shot configuration
    private static final int BALLS_PER_POSITION = 3;  // Shoot 3 balls at each position

    // Shooter state tracking
    private double currentTargetRPM = 0;
    private boolean shooterVelocityReached = false;
    private int currentShotNumber = 0;
    private int ballsShot = 0;  // Track balls shot at current position

    // AprilTag tracking state (for turret aiming and shooter power)
    private boolean limelightHasTarget = false;
    private double limelightDistance = 0;
    private double limelightTx = 0;
    private double turretPosition = TURRET_CENTER;
    private double autoPowerLevel = FALLBACK_SHOOTER_POWER;

    // ==================== Poses ====================
    // Start position
    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));

    // Shooting position
    private final Pose shoot1Pose = new Pose(85.07215815485998, 17.555189456342667, Math.toRadians(65));

    // Waypoint (end position)
    private final Pose waypoint1Pose = new Pose(108.07742998352553, 11.426688632619447, Math.toRadians(90));

    // ==================== Path Chains ====================
    private PathChain path1_toShoot1;
    private PathChain path2_toWaypoint1;

    public void buildPaths() {
        // Path 1: Start -> Shoot 1
        path1_toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                .build();

        // Path 2: Shoot 1 -> Waypoint 1 (end position)
        path2_toWaypoint1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, waypoint1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))
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
     * Start shooter with auto power based on limelight distance.
     * Falls back to FALLBACK_SHOOTER_POWER if no target detected.
     */
    private void startShooter(int shotNumber) {
        currentShotNumber = shotNumber;
        shooterVelocityReached = false;

        double power = limelightHasTarget ? autoPowerLevel : FALLBACK_SHOOTER_POWER;

        currentTargetRPM = power * ShooterConfig.MOTOR_FREE_SPEED_RPM;
    }

    private void updateShooterPower() {
        if (currentTargetRPM <= 0) return;
        double power = limelightHasTarget ? autoPowerLevel : FALLBACK_SHOOTER_POWER;
        currentTargetRPM = power * ShooterConfig.MOTOR_FREE_SPEED_RPM;
    }

    private void updateShooterOutput() {
        if (currentTargetRPM <= 0) { shooter.setPower(0); return; }
        double actualRPM = (shooter.getVelocity() / ShooterConfig.TICKS_PER_REV) * 60.0;
        double feedForward = ShooterConfig.KV_INITIAL * currentTargetRPM + ShooterConfig.KS_INITIAL;
        double pTerm = ShooterConfig.KP_INITIAL * (currentTargetRPM - actualRPM);
        shooter.setPower(Math.max(0.0, Math.min(1.0, feedForward + pTerm)));
    }

    private void updateShooterVelocityStatus() {
        if (currentTargetRPM <= 0) { shooterVelocityReached = false; return; }
        double actualRPM = (shooter.getVelocity() / ShooterConfig.TICKS_PER_REV) * 60.0;
        shooterVelocityReached = Math.abs(currentTargetRPM - actualRPM) <= ShooterConfig.VELOCITY_TOLERANCE_RPM;
    }

    private void stopShooter() {
        shooter.setPower(0);
        currentTargetRPM = 0;
        shooterVelocityReached = false;
    }

    private void stopAllMechanisms() {
        stopIntake();
        stopShooter();
    }

    // ==================== Limelight AprilTag Tracking ====================

    /**
     * Updates AprilTag tracking data from Limelight.
     * Gets distance for power calculation and tx for turret aiming.
     */
    private void updateLimelightTracking() {
        limelightHasTarget = false;
        limelightDistance = 0;
        limelightTx = 0;

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
            if (TARGET_APRILTAG_ID == -1) {
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
        if (distance <= DISTANCE_PRESETS[0]) {
            return POWER_PRESETS[0];
        }
        if (distance >= DISTANCE_PRESETS[DISTANCE_PRESETS.length - 1]) {
            return POWER_PRESETS[POWER_PRESETS.length - 1];
        }

        for (int i = 0; i < DISTANCE_PRESETS.length - 1; i++) {
            if (distance >= DISTANCE_PRESETS[i] && distance <= DISTANCE_PRESETS[i + 1]) {
                double t = (distance - DISTANCE_PRESETS[i]) /
                        (DISTANCE_PRESETS[i + 1] - DISTANCE_PRESETS[i]);
                return POWER_PRESETS[i] + t * (POWER_PRESETS[i + 1] - POWER_PRESETS[i]);
            }
        }

        return POWER_PRESETS[0];
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
            telemetry.addData("Distance", String.format(Locale.US, "%.1f in", limelightDistance));
            telemetry.addData("TX (offset)", String.format(Locale.US, "%.1f deg", limelightTx));
            telemetry.addData("Turret Pos", String.format(Locale.US, "%.3f", turretPosition));
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
        double actualRPM = (shooter.getVelocity() / ShooterConfig.TICKS_PER_REV) * 60.0;

        telemetry.addLine("─── SHOOTER (KSV) ───");
        telemetry.addData("Position", currentShotNumber);
        telemetry.addData("Balls Shot", String.format(Locale.US, "%d / %d", ballsShot, BALLS_PER_POSITION));
        telemetry.addData("Power Level", String.format(Locale.US, "%.0f%% (%s)",
                autoPowerLevel * 100, limelightHasTarget ? "auto" : "fallback"));
        telemetry.addData("Actual RPM", String.format(Locale.US, "%.0f", actualRPM));
        telemetry.addData("Target RPM", String.format(Locale.US, "%.0f", currentTargetRPM));
        telemetry.addData("Motor Output", String.format(Locale.US, "%.3f", shooter.getPower()));

        if (currentTargetRPM > 0) {
            telemetry.addData("RPM Error", String.format(Locale.US, "%.0f", currentTargetRPM - actualRPM));
            telemetry.addData("Ready", shooterVelocityReached ? "YES" : "NO");
        }
    }

    // ==================== State Machine ====================
    // Simplified: Start -> Shoot1 (3 balls) -> Waypoint1 (end)

    public void autonomousPathUpdate() {
        // Update KSV controller and velocity status every loop
        updateShooterOutput();
        updateShooterVelocityStatus();

        switch (pathState) {
            // ===== Drive to Shoot Position 1 =====
            case 0:
                follower.followPath(path1_toShoot1, true);
                // Start shooter while driving to allow motor to reach target RPM
                updateLimelightTracking();
                startShooter(1);
                setPathState(1);
                break;

            // ===== Score at Position 1 (3 balls) =====
            case 1:
                if (!follower.isBusy()) {
                    // Shooter already spinning - just update tracking and aim
                    updateLimelightTracking();
                    aimTurretWithTracking();
                    updateShooterPower();  // Adjust power based on limelight distance
                    ballsShot = 0;
                    setPathState(2);
                }
                break;

            case 2:
                // Wait 0.5s minimum for shooter to stabilize, then check velocity OR timeout
                updateLimelightTracking();
                aimTurretWithTracking();
                updateShooterPower();  // Keep adjusting power as limelight refines
                if (actionTimer.getElapsedTimeSeconds() > 0.5 &&
                        (shooterVelocityReached ||
                                actionTimer.getElapsedTimeSeconds() > MAX_SPINUP_TIME)) {
                    startIntake();
                    setPathState(3);
                }
                break;

            case 3:
                // Push complete
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    ballsShot++;
                    setPathState(4);
                }
                break;

            case 4:
                // Check if more balls to shoot at this position
                if (actionTimer.getElapsedTimeSeconds() > RETRACT_TIME) {
                    if (ballsShot < BALLS_PER_POSITION) {
                        setPathState(3);  // Pusher still running
                    } else {
                        // Done shooting, move to waypoint 1 (end position)
                        stopIntake();
                        stopShooter();
                        follower.followPath(path2_toWaypoint1, true);
                        setPathState(5);
                    }
                }
                break;

            // ===== Drive to Waypoint 1 (End) =====
            case 5:
                if (!follower.isBusy()) {
                    // All done!
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

        // Initialize Pedro Pathing follower (Pinpoint localization ONLY)
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        // Initialize intake motors
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearIntake = hardwareMap.get(DcMotor.class, "rearIntake");
        rearIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize shooter motor with KSV open-loop control
        shooter = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR_1_NAME);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR_2_NAME);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        turretGear = hardwareMap.get(Servo.class, "turretGear");
        turretGear.setPosition(TURRET_CENTER);
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

        // Initialize indicator LED
        indicator = hardwareMap.get(Servo.class, "indicator");
        indicator.setPosition(INDICATOR_OFF);

        // Build paths
        buildPaths();

        // Display initialization info
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("RedQuick - MINIMAL AUTO");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("Localization: Pinpoint ONLY");
        telemetry.addLine("Limelight: Tracking & Power");
        telemetry.addData("Balls per Position", BALLS_PER_POSITION);
        telemetry.addLine("Shooter spins up during drive");
        telemetry.addLine("────────────────────────────────");
        telemetry.addData("Shooter Power", "Auto (limelight distance)");
        telemetry.addData("Fallback Power", String.format(Locale.US, "%.0f%%", FALLBACK_SHOOTER_POWER * 100));
        telemetry.addData("Limelight", limelightConnected ? "Connected (turret + power)" : "NOT FOUND");
        telemetry.addData("Battery", String.format(Locale.US, "%.2fV",
                hardwareMap.voltageSensor.iterator().next().getVoltage()));
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

        // Update indicator LED: green when AprilTag is detected
        indicator.setPosition(limelightHasTarget ? INDICATOR_GREEN : INDICATOR_OFF);

        // Run state machine
        autonomousPathUpdate();

        // Telemetry - Robot State
        telemetry.addData("Path State", pathState);
        telemetry.addData("Opmode Time", String.format(Locale.US, "%.1f s", opmodeTimer.getElapsedTimeSeconds()));

        // Telemetry - Pinpoint Position (ONLY localization source)
        telemetry.addLine("─── PINPOINT POSITION ───");
        telemetry.addData("X", String.format(Locale.US, "%.1f in", follower.getPose().getX()));
        telemetry.addData("Y", String.format(Locale.US, "%.1f in", follower.getPose().getY()));
        telemetry.addData("Heading", String.format(Locale.US, "%.1f deg", Math.toDegrees(follower.getPose().getHeading())));

        // Telemetry - Shooter
        addShooterTelemetry();

        // Telemetry - Limelight Tracking
        addLimelightTelemetry();

        // Telemetry - Mechanisms
        telemetry.addLine("─── MECHANISMS ───");
        telemetry.addData("Intake", frontIntake.getPower() > 0 ? "ON" : "OFF");

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelightConnected) {
            limelight.stop();
        }
        indicator.setPosition(INDICATOR_OFF);
        stopAllMechanisms();
    }
}
