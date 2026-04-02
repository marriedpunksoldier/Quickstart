package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

/**
 * Red Alliance TeleOp with Limelight auto-distance targeting.
 *
 * Limelight API Reference: <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming">...</a>
 *
 * Limelight API Usage Summary:
 * - Initialization: hardwareMap.get(Limelight3A.class, "limelight")
 * - setPollRateHz(100): Set query frequency (recommended 100Hz)
 * - pipelineSwitch(n): Select vision pipeline 0-9 (non-blocking)
 * - start() / stop(): Activate/deactivate vision processing
 * - getLatestResult(): Get LLResult with all detection data
 *
 * LLResult Methods:
 * - isValid(): Check if result contains valid data
 * - getStaleness(): Data age in milliseconds
 * - getPipelineIndex(): Current active pipeline
 * - getTx()/getTy()/getTa(): Basic targeting data (degrees/area)
 * - getFiducialResults(): List of detected AprilTags
 * - getBotpose(): MegaTag 1 robot pose in field coordinates
 * - getBotpose_MT2(): MegaTag 2 pose (requires IMU fusion via updateRobotOrientation)
 *
 * FiducialResult Methods (AprilTag data):
 * - getFiducialId(): Tag ID number
 * - getTargetXDegrees()/getTargetYDegrees(): Horizontal/vertical offset
 * - getRobotPoseTargetSpace(): Robot pose relative to tag
 * - getCameraPoseTargetSpace(): Camera pose relative to tag
 * - getRobotPoseFieldSpace(): Robot pose in field coordinates
 */

@TeleOp(name = "Red Teleop v2", group = "TeleOp")
public class RedTeleopv2 extends OpMode {
    // ═══════════════════════════════════════════════════════════════════
    // DISTANCE-POWER LOOKUP TABLE
    // ═══════════════════════════════════════════════════════════════════
    // Distances in inches
    private static final double[] DISTANCE_PRESETS = {24.0, 36.0, 48.0, 60.0, 72.0, 84.0, 120.0, 132.0};
    // Corresponding power levels (0.0 - 1.0)
    private static final double[] POWER_PRESETS =   {0.50, 0.50, 0.50, 0.55, 0.58, 0.60, 0.70, 0.75};
    // Distance preset names for display
    private static final String[] DISTANCE_NAMES = {"2 ft", "3 ft", "4 ft", "5 ft", "6 ft", "7 ft", "10 ft", "11 ft"};

    private int currentDistanceIndex = 0;  // Start at 24"

    // ═══════════════════════════════════════════════════════════════════
    // AUTO-MOVE TO DISTANCE (B BUTTON)
    // ═══════════════════════════════════════════════════════════════════
    private static final double AUTO_MOVE_TARGET_DISTANCE = 55.0;  // inches from AprilTag
    private static final double AUTO_MOVE_SHOOTER_POWER = 0.50;    // power at target distance

    // Auto-move state
    private enum AutoMoveState { IDLE, MOVING, AT_DISTANCE }
    private AutoMoveState autoMoveState = AutoMoveState.IDLE;

    // ═══════════════════════════════════════════════════════════════════
    // AUTO-DISTANCE MODE
    // ═══════════════════════════════════════════════════════════════════
    private boolean autoDistanceMode = true;  // Default AUTO, Y button toggles to manual
    private double limelightDistance = 0;      // Distance from Limelight in inches
    private double autoPowerLevel = 0;         // Interpolated power level
    private boolean limelightHasTarget = false;
    private double limelightTx = 0;            // Horizontal offset in degrees (tx from Limelight)
    private double limelightTy = 0;            // Vertical offset in degrees (ty from Limelight)

    // Target AprilTag ID for distance calculation (set to your scoring target)
    private static final int TARGET_APRILTAG_ID = 24;  // -1 = use any/closest tag

    // ═══════════════════════════════════════════════════════════════════
    // LIMELIGHT DISTANCE ESTIMATION CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════
    // Reference: https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
    //
    // Two methods are available for distance estimation:
    //
    // METHOD 1: AprilTag 3D Pose (Default)
    //   - Uses AprilTag's built-in 3D pose estimation
    //   - Calculates Euclidean distance from camera to tag center
    //   - More accurate when AprilTags are detected
    //   - Works regardless of camera mounting angle
    //
    // METHOD 2: Trigonometric (Fixed Angle Camera)
    //   - Uses formula: d = (h2 - h1) / tan(a1 + a2)
    //   - h1 = Camera lens height above floor (LIMELIGHT_HEIGHT_INCHES)
    //   - h2 = Target height above floor (TARGET_HEIGHT_INCHES)
    //   - a1 = Camera mounting angle in degrees (LIMELIGHT_MOUNT_ANGLE_DEGREES)
    //   - a2 = Vertical angle to target (ty value from Limelight)
    //   - d  = Horizontal distance to target
    //   - Best when camera angle is fixed and target/camera heights differ significantly
    //
    // Set USE_TRIGONOMETRIC_DISTANCE to true to use Method 2 instead of Method 1
    // ═══════════════════════════════════════════════════════════════════

    // Distance estimation method selection
    private static final boolean USE_TRIGONOMETRIC_DISTANCE = false;  // false = AprilTag 3D Pose, true = Trigonometric

    // Trigonometric method constants (only used if USE_TRIGONOMETRIC_DISTANCE = true)
    // IMPORTANT: Measure and update these values for your specific robot setup!
    private static final double LIMELIGHT_HEIGHT_INCHES = 13.0;       // h1: Camera lens height above floor
    private static final double TARGET_HEIGHT_INCHES = 29.5;          // h2: AprilTag center height above floor
    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 27;  // a1: Camera angle (0° = horizontal, positive = tilted up)

    // Turret direct angle mapping for goBILDA Super Speed servo at 90° range
    // 90 degrees across full servo range (0.0 to 1.0), so 90 degrees per unit
    private static final double TURRET_DEGREES_PER_SERVO_UNIT = 90.0;

    // Turret lock-on state
    private boolean turretLockedOn = false;
    private double lastKnownTx = 0;  // Last known target offset when locked

    // ═══════════════════════════════════════════════════════════════════

    // Pedro Pathing (for field-centric drive)
    private Follower follower;
    private Timer opmodeTimer;

    // Hardware - Drive motors handled by Pedro Pathing
    private DcMotor frontIntake;
    private DcMotorEx shooter;
    private Servo turretGear;
    private CRServo pusherServo;

    // Limelight
    private Limelight3A limelight;
    private boolean limelightConnected = false;

    // Indicator LED
    private Servo indicator;
    private static final double INDICATOR_GREEN = 0.500;
    private static final double INDICATOR_OFF   = 0.28;

    // ═══════════════════════════════════════════════════════════════════
    // SHOOTER CONFIGURATION (from BlueAutov3)
    // ═══════════════════════════════════════════════════════════════════

    // Motor specifications (goBILDA 5203 series 1:1 ratio)
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double TICKS_PER_SECOND_AT_MAX_RPM = (MOTOR_MAX_RPM / 60.0) * MOTOR_TICKS_PER_REV;

    // PIDF coefficients for velocity control
    private static final double SHOOTER_KP = 8.00;
    private static final double SHOOTER_KI = 0.000;
    private static final double SHOOTER_KD = 0.10;
    private static final double SHOOTER_KF = 13.00;

    // Velocity verification
    private static final double VELOCITY_TOLERANCE_PERCENT = 2.0;

    // ═══════════════════════════════════════════════════════════════════

    // Servo positions and motor powers
    private static final double INTAKE_POWER = 1.0;
    private static final double TURRET_CENTER = 0.5;
    private static final double TURRET_MIN = 0.0;
    private static final double TURRET_MAX = 1.0;
    private static final double PUSHER_FORWARD_POWER = 1.0;
    private static final double PUSHER_REVERSE_POWER = -1.0;

    // Limelight settings
    // Reference: https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
    private static final int LIMELIGHT_PIPELINE = 2;  // Set to your vision pipeline index (0-9)
    private static final int LIMELIGHT_POLL_RATE_HZ = 100;  // How often to query Limelight (recommended: 100Hz)
    private static final double LIMELIGHT_MAX_STALENESS_MS = 100.0;  // Max age of data before considered stale

    // Shooter state tracking
    private double currentTargetVelocity = 0;
    private boolean shooterRunning = false;
    private boolean shooterVelocityReached = false;

    // Turret position
    private double turretPosition = TURRET_CENTER;
    private static final double TURRET_INCREMENT = 0.02;

    // Button debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRightTrigger = false;
    private boolean lastYButton = false;
    private boolean lastBButton = false;

    // ═══════════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ═══════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        opmodeTimer = new Timer();

        // Initialize Pedro Pathing follower for field-centric drive
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, Math.toRadians(0)));
        follower.startTeleopDrive();

        // Initialize intake motors
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize shooter motor with PIDF velocity control
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // PIDF velocity control

        // Set PIDF coefficients for velocity control
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
                SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF
        );
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Initialize servos
        turretGear = hardwareMap.get(Servo.class, "turretGear");
        pusherServo = hardwareMap.get(CRServo.class, "pusherServo");
        turretGear.setPosition(TURRET_CENTER);
        pusherServo.setPower(0);
        turretPosition = TURRET_CENTER;

        // Initialize Limelight
        // Reference: https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(LIMELIGHT_POLL_RATE_HZ);  // Set query frequency (recommended: 100Hz)
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);     // Select vision pipeline (non-blocking)
            limelight.start();                                 // Activate vision processing
            limelightConnected = true;
        } catch (Exception e) {
            limelightConnected = false;
        }

        // Initialize indicator LED
        indicator = hardwareMap.get(Servo.class, "indicator");
        indicator.setPosition(INDICATOR_OFF);

        // Display initialization
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("RED TELEOP v2 - AUTO DISTANCE");
        telemetry.addLine("════════════════════════════════");

        // Show distance estimation method configuration
        telemetry.addLine("─── Distance Estimation ───");
        if (USE_TRIGONOMETRIC_DISTANCE) {
            telemetry.addData("Method", "Trigonometric");
            telemetry.addData("  Camera Height", String.format(Locale.US, "%.1f\"", LIMELIGHT_HEIGHT_INCHES));
            telemetry.addData("  Target Height", String.format(Locale.US, "%.1f\"", TARGET_HEIGHT_INCHES));
            telemetry.addData("  Mount Angle", String.format(Locale.US, "%.1f°", LIMELIGHT_MOUNT_ANGLE_DEGREES));
        } else {
            telemetry.addData("Method", "AprilTag 3D Pose");
        }

        telemetry.addLine("\n─── Distance-Power Table ───");
        for (int i = 0; i < DISTANCE_PRESETS.length; i++) {
            telemetry.addData("  " + DISTANCE_NAMES[i],
                    String.format(Locale.US, "%.0f%% power", POWER_PRESETS[i] * 100));
        }
        telemetry.addLine("\n─── Auto-Move ───");
        telemetry.addData("  Target Distance", String.format(Locale.US, "%.0f\"", AUTO_MOVE_TARGET_DISTANCE));
        telemetry.addData("  Shooter Power", String.format(Locale.US, "%.0f%%", AUTO_MOVE_SHOOTER_POWER * 100));
        telemetry.addData("  Trigger", "B button (GP1)");
        telemetry.addLine("────────────────────────────────");
        telemetry.addData("Limelight", limelightConnected ? "Connected" : "NOT FOUND");
        telemetry.addData("Auto-Distance", "Press Y to toggle");
        telemetry.addLine("Ready to Start");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    // ═══════════════════════════════════════════════════════════════════
    // MAIN LOOP
    // ═══════════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        // Update Limelight distance first
        updateLimelightDistance();

        // Update indicator LED: green when AprilTag is detected
        indicator.setPosition(limelightHasTarget ? INDICATOR_GREEN : INDICATOR_OFF);

        // Handle auto-move (B button) - must run before drive
        handleAutoMove();

        // Drive and shooter controls are disabled while auto-moving
        if (autoMoveState != AutoMoveState.MOVING) {
            handleDrive();
        }
        if (autoMoveState == AutoMoveState.IDLE) {
            handleAutoDistanceToggle();
            handleDistanceSelection();
            handleShooter();
        }

        // Pusher, intake, and turret always available
        handlePusher();
        handleIntake();
        handleTurret();

        // Update shooter velocity status
        updateShooterVelocityStatus();

        // Update telemetry
        displayTelemetry();
    }

    // ═══════════════════════════════════════════════════════════════════
    // LIMELIGHT DISTANCE CALCULATION
    // ═══════════════════════════════════════════════════════════════════

    /**
     * Updates the distance to target from Limelight AprilTag detection.
     *
     * Supports two distance estimation methods per Limelight documentation:
     * https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
     *
     * METHOD 1 (Default): AprilTag 3D Pose
     *   - Uses getRobotPoseTargetSpace() to get the tag's 3D position relative to camera
     *   - Calculates Euclidean distance: sqrt(x² + y² + z²)
     *   - Converts from meters to inches (multiply by 39.3701)
     *   - Most accurate for AprilTag detection, works at any camera angle
     *
     * METHOD 2: Trigonometric (Fixed Angle Camera)
     *   - Formula: d = (h2 - h1) / tan(a1 + a2)
     *   - Uses ty (vertical angle offset) from Limelight
     *   - Requires accurate camera height, target height, and mount angle configuration
     *   - Best when camera mount angle is fixed and height difference is significant
     *
     * The method is selected via USE_TRIGONOMETRIC_DISTANCE constant.
     */
    private void updateLimelightDistance() {
        limelightHasTarget = false;
        limelightDistance = 0;
        autoPowerLevel = 0;
        limelightTx = 0;
        limelightTy = 0;

        if (!limelightConnected) {
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        // Check data staleness - getStaleness() returns age in milliseconds
        // Stale data may be inaccurate if robot has moved since capture
        if (result.getStaleness() > LIMELIGHT_MAX_STALENESS_MS) {
            return;  // Data too old, skip this frame
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            return;
        }

        // Find the target AprilTag (or use closest if TARGET_APRILTAG_ID is -1)
        LLResultTypes.FiducialResult targetTag = null;
        double closestDistance = Double.MAX_VALUE;

        for (LLResultTypes.FiducialResult fr : fiducials) {
            double distance;

            if (USE_TRIGONOMETRIC_DISTANCE) {
                // ═══════════════════════════════════════════════════════════
                // METHOD 2: Trigonometric Distance Estimation
                // ═══════════════════════════════════════════════════════════
                // Formula: d = (h2 - h1) / tan(a1 + a2)
                // Reference: https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
                //
                // Variables:
                //   h1 = LIMELIGHT_HEIGHT_INCHES (camera lens height above floor)
                //   h2 = TARGET_HEIGHT_INCHES (target center height above floor)
                //   a1 = LIMELIGHT_MOUNT_ANGLE_DEGREES (camera mounting angle, 0° = horizontal)
                //   a2 = ty value (vertical angle to target from Limelight)
                //   d  = horizontal distance to target
                //
                // Note: This method is ineffective when target and camera heights are nearly equal
                // ═══════════════════════════════════════════════════════════

                double ty = fr.getTargetYDegrees();  // a2: Vertical angle to target

                // Calculate total angle to target (a1 + a2)
                double angleToTargetDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
                double angleToTargetRadians = Math.toRadians(angleToTargetDegrees);

                // Avoid division by zero or very small angles
                if (Math.abs(Math.tan(angleToTargetRadians)) < 0.001) {
                    continue;
                }

                // Calculate horizontal distance: d = (h2 - h1) / tan(a1 + a2)
                distance = (TARGET_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES) / Math.tan(angleToTargetRadians);

                // Distance must be positive (target should be in front of camera)
                if (distance <= 0) {
                    continue;
                }
            } else {
                // ═══════════════════════════════════════════════════════════
                // METHOD 1: AprilTag 3D Pose Distance Estimation (Default)
                // ═══════════════════════════════════════════════════════════
                // Uses the AprilTag's built-in 3D pose estimation
                // getRobotPoseTargetSpace() returns the tag's position relative to the camera
                // Distance is calculated as 3D Euclidean distance: sqrt(x² + y² + z²)
                //
                // Advantages:
                //   - Works regardless of camera mounting angle
                //   - More accurate when AprilTags are clearly visible
                //   - Accounts for all three dimensions
                //
                // The pose coordinates are in meters, so we convert to inches (* 39.3701)
                // ═══════════════════════════════════════════════════════════

                Pose3D tagPose = fr.getRobotPoseTargetSpace();
                if (tagPose == null) {
                    continue;
                }

                // Get 3D position and convert meters to inches
                double x = tagPose.getPosition().x * 39.3701;  // Left/right offset
                double y = tagPose.getPosition().y * 39.3701;  // Up/down offset
                double z = tagPose.getPosition().z * 39.3701;  // Forward distance

                // Calculate 3D Euclidean distance
                distance = Math.sqrt(x * x + y * y + z * z);
            }

            // Check if this is our target tag or the closest one
            if (TARGET_APRILTAG_ID == -1) {
                // Use closest tag when TARGET_APRILTAG_ID is -1
                if (distance < closestDistance) {
                    closestDistance = distance;
                    targetTag = fr;
                    limelightDistance = distance;
                }
            } else if (fr.getFiducialId() == TARGET_APRILTAG_ID) {
                // Found our specific target tag
                targetTag = fr;
                limelightDistance = distance;
                break;
            }
        }

        if (targetTag != null && limelightDistance > 0) {
            limelightHasTarget = true;
            autoPowerLevel = interpolatePower(limelightDistance);

            // Get angle offsets for turret tracking
            // tx: Horizontal offset in degrees (positive = target is to the right)
            // ty: Vertical offset in degrees (positive = target is above crosshair)
            limelightTx = targetTag.getTargetXDegrees();
            limelightTy = targetTag.getTargetYDegrees();
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

        // Find the two closest presets
        for (int i = 0; i < DISTANCE_PRESETS.length - 1; i++) {
            if (distance >= DISTANCE_PRESETS[i] && distance <= DISTANCE_PRESETS[i + 1]) {
                // Linear interpolation
                double t = (distance - DISTANCE_PRESETS[i]) /
                        (DISTANCE_PRESETS[i + 1] - DISTANCE_PRESETS[i]);
                return POWER_PRESETS[i] + t * (POWER_PRESETS[i + 1] - POWER_PRESETS[i]);
            }
        }

        // Fallback
        return POWER_PRESETS[0];
    }

    // ═══════════════════════════════════════════════════════════════════
    // DRIVE CONTROL
    // ═══════════════════════════════════════════════════════════════════

    private void handleDrive() {
        // Get joystick inputs (field-centric via Pedro Pathing)
        // Negate Y axis because pushing forward gives negative values
        double forward = -gamepad1.left_stick_y;   // Forward/backward
        double strafe = -gamepad1.left_stick_x;    // Left/right
        double rotate = -gamepad1.right_stick_x;   // Rotation

        // Apply deadzone
        if (Math.abs(forward) < 0.05) forward = 0;
        if (Math.abs(strafe) < 0.05) strafe = 0;
        if (Math.abs(rotate) < 0.05) rotate = 0;

        // Update Pedro Pathing teleop drive (forward, strafe, rotate, fieldCentric)
        follower.setTeleOpDrive(forward, strafe, rotate, true);
        follower.update();
    }

    // ═══════════════════════════════════════════════════════════════════
    // AUTO-MOVE TO DISTANCE - GAMEPAD 1 (B BUTTON)
    // ═══════════════════════════════════════════════════════════════════

    private void handleAutoMove() {
        boolean bButton = gamepad1.b;
        boolean bPressed = bButton && !lastBButton;

        switch (autoMoveState) {
            case IDLE:
                if (bPressed && limelightHasTarget && limelightDistance > 0) {
                    startAutoMove();
                }
                break;

            case MOVING:
                follower.update();

                if (!follower.isBusy()) {
                    autoMoveState = AutoMoveState.AT_DISTANCE;
                    follower.startTeleopDrive();
                }

                if (bPressed || hasSignificantDriverInput()) {
                    cancelAutoMove();
                }
                break;

            case AT_DISTANCE:
                if (bPressed) {
                    cancelAutoMove();
                }
                break;
        }

        lastBButton = bButton;
    }

    private void startAutoMove() {
        Pose currentPose = follower.getPose();
        double currentHeading = currentPose.getHeading();

        double directionToTag = currentHeading - Math.toRadians(limelightTx);
        double delta = limelightDistance - AUTO_MOVE_TARGET_DISTANCE;

        double targetX = currentPose.getX() + delta * Math.cos(directionToTag);
        double targetY = currentPose.getY() + delta * Math.sin(directionToTag);

        Pose startPoint = new Pose(currentPose.getX(), currentPose.getY(), currentHeading);
        Pose endPoint = new Pose(targetX, targetY, currentHeading);

        PathChain movePath = follower.pathBuilder()
                .addPath(new BezierLine(startPoint, endPoint))
                .setLinearHeadingInterpolation(currentHeading, currentHeading)
                .build();

        follower.followPath(movePath, true);

        currentTargetVelocity = AUTO_MOVE_SHOOTER_POWER * TICKS_PER_SECOND_AT_MAX_RPM;
        shooter.setVelocity(currentTargetVelocity);
        shooterRunning = true;
        shooterVelocityReached = false;

        autoMoveState = AutoMoveState.MOVING;
    }

    private void cancelAutoMove() {
        autoMoveState = AutoMoveState.IDLE;
        follower.startTeleopDrive();
        stopShooter();
    }

    private boolean hasSignificantDriverInput() {
        return Math.abs(gamepad1.left_stick_x) > 0.3 ||
               Math.abs(gamepad1.left_stick_y) > 0.3 ||
               Math.abs(gamepad1.right_stick_x) > 0.3;
    }

    // ═══════════════════════════════════════════════════════════════════
    // AUTO-DISTANCE TOGGLE - GAMEPAD 2 (Y BUTTON)
    // ═══════════════════════════════════════════════════════════════════

    private void handleAutoDistanceToggle() {
        boolean yButton = gamepad2.y;

        if (yButton && !lastYButton) {
            autoDistanceMode = !autoDistanceMode;
        }

        lastYButton = yButton;
    }

    // ═══════════════════════════════════════════════════════════════════
    // DISTANCE SELECTION - GAMEPAD 2 (D-PAD UP/DOWN)
    // ═══════════════════════════════════════════════════════════════════

    private void handleDistanceSelection() {
        // Only allow manual selection when not in auto mode
        if (autoDistanceMode) {
            return;
        }

        boolean dpadUp = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;

        // D-pad Up: Next distance preset
        if (dpadUp && !lastDpadUp) {
            currentDistanceIndex = (currentDistanceIndex + 1) % DISTANCE_PRESETS.length;
        }

        // D-pad Down: Previous distance preset
        if (dpadDown && !lastDpadDown) {
            currentDistanceIndex = (currentDistanceIndex - 1 + DISTANCE_PRESETS.length) % DISTANCE_PRESETS.length;
        }

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
    }

    // ═══════════════════════════════════════════════════════════════════
    // SHOOTER CONTROL - GAMEPAD 2 (RIGHT BUMPER)
    // ═══════════════════════════════════════════════════════════════════

    private void handleShooter() {
        // Right bumper: Spin up shooter at selected power level
        if (gamepad2.right_bumper) {
            if (!shooterRunning) {
                startShooter();
            } else if (autoDistanceMode && limelightHasTarget) {
                // Continuously update velocity in auto mode if target is visible
                updateShooterVelocityForAuto();
            }
        } else {
            if (shooterRunning) {
                stopShooter();
            }
        }
    }

    private void startShooter() {
        double powerLevel;

        if (autoDistanceMode && limelightHasTarget) {
            // Use auto-calculated power from Limelight distance
            powerLevel = autoPowerLevel;
        } else {
            // Use manual preset
            powerLevel = POWER_PRESETS[currentDistanceIndex];
        }

        // PIDF velocity control
        currentTargetVelocity = powerLevel * TICKS_PER_SECOND_AT_MAX_RPM;
        shooter.setVelocity(currentTargetVelocity);
        shooterRunning = true;
        shooterVelocityReached = false;
    }

    private void updateShooterVelocityForAuto() {
        // Update velocity based on current Limelight distance
        if (limelightHasTarget && autoPowerLevel > 0) {
            double newTargetVelocity = autoPowerLevel * TICKS_PER_SECOND_AT_MAX_RPM;
            // Only update if significantly different (>2% change)
            if (Math.abs(newTargetVelocity - currentTargetVelocity) / currentTargetVelocity > 0.02) {
                currentTargetVelocity = newTargetVelocity;
                shooter.setVelocity(currentTargetVelocity);  // PIDF velocity control
            }
        }
    }

    private void stopShooter() {
        shooter.setVelocity(0);
        currentTargetVelocity = 0;
        shooterRunning = false;
        shooterVelocityReached = false;
    }

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

    // ═══════════════════════════════════════════════════════════════════
    // PUSHER CONTROL - GAMEPAD 2 (TRIGGERS)
    // ═══════════════════════════════════════════════════════════════════

    private void handlePusher() {
        boolean rightTrigger = gamepad2.right_trigger > 0.5;
        boolean leftTrigger = gamepad2.left_trigger > 0.5;

        // Right trigger: Forward, Left trigger: Reverse, Neither: Stop
        if (rightTrigger) {
            pusherServo.setPower(PUSHER_FORWARD_POWER);
        } else if (leftTrigger) {
            pusherServo.setPower(PUSHER_REVERSE_POWER);
        } else {
            pusherServo.setPower(0);
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // INTAKE CONTROL - GAMEPAD 2 (LEFT BUMPER / LEFT TRIGGER)
    // ═══════════════════════════════════════════════════════════════════

    private void handleIntake() {
        // Left bumper: Run intake forward
        if (gamepad2.left_bumper) {
            frontIntake.setPower(INTAKE_POWER);
        }
        // Left trigger: Reverse intake
        else if (gamepad2.left_trigger > 0.5) {
            frontIntake.setPower(-INTAKE_POWER);
        }
        // No input: Stop intake
        else {
            frontIntake.setPower(0);
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // TURRET CONTROL - GAMEPAD 2 (D-PAD LEFT/RIGHT)
    // ═══════════════════════════════════════════════════════════════════

    private void handleTurret() {
        // D-pad Left/Right: Manual turret adjustment (breaks lock and overrides auto-tracking)
        if (gamepad2.dpad_left) {
            turretPosition = Math.max(TURRET_MIN, turretPosition - TURRET_INCREMENT);
            turretLockedOn = false;  // Manual control breaks lock
        } else if (gamepad2.dpad_right) {
            turretPosition = Math.min(TURRET_MAX, turretPosition + TURRET_INCREMENT);
            turretLockedOn = false;  // Manual control breaks lock
        }
        // Auto-track AprilTag when in auto mode
        else if (autoDistanceMode) {
            // Update lock state and tracking
            if (limelightHasTarget) {
                // Target visible - lock on and track
                turretLockedOn = true;
                lastKnownTx = limelightTx;

                // Calculate target servo position
                double servoOffset = limelightTx / TURRET_DEGREES_PER_SERVO_UNIT;
                double targetPosition = TURRET_CENTER - servoOffset;

                // Check if target is within servo range
                if (targetPosition >= TURRET_MIN && targetPosition <= TURRET_MAX) {
                    // Target is in range - track it
                    turretPosition = targetPosition;
                } else {
                    // Target moved out of servo range - break lock
                    turretLockedOn = false;
                    // Clamp to limit so turret stays at edge
                    turretPosition = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetPosition));
                }
            } else if (turretLockedOn) {
                // Target not visible but we're locked on - maintain last position
                // The turret stays where it was, waiting for target to reappear
                // Lock only breaks if:
                // 1. Target reappears outside servo range
                // 2. Manual control is used (D-pad)
                // 3. A button is pressed to center
            }
            // If not locked and no target, turret stays at current position
        }

        // A button: Center turret and break lock
        if (gamepad2.a) {
            turretPosition = TURRET_CENTER;
            turretLockedOn = false;
        }

        turretGear.setPosition(turretPosition);
    }

    // ═══════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════

    private void displayTelemetry() {
        // Header
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("RED TELEOP v2");
        telemetry.addLine("═══════════════════════════════════════");

        // Auto-Move Status
        if (autoMoveState != AutoMoveState.IDLE) {
            telemetry.addLine("\n─── AUTO-MOVE ───");
            telemetry.addData("Status", autoMoveState == AutoMoveState.MOVING ?
                    "MOVING TO TARGET" : "AT DISTANCE - READY TO SHOOT");
            telemetry.addData("Shooter Power", String.format(Locale.US, "%.0f%%", AUTO_MOVE_SHOOTER_POWER * 100));
            if (limelightHasTarget) {
                telemetry.addData("Current Distance", String.format(Locale.US, "%.1f\"", limelightDistance));
            }
            telemetry.addLine("Press B to cancel");
        }

        // Auto-Distance Mode Status
        telemetry.addLine("\n─── DISTANCE MODE ───");
        telemetry.addData("Mode", autoDistanceMode ? "AUTO (Limelight)" : "MANUAL");
        telemetry.addLine("(Press Y to toggle)");

        if (autoDistanceMode) {
            // Show distance estimation method being used
            telemetry.addData("Method", USE_TRIGONOMETRIC_DISTANCE ? "Trigonometric" : "AprilTag 3D Pose");

            // Show Limelight auto-distance info
            if (limelightHasTarget) {
                telemetry.addData("Detected Distance", String.format(Locale.US, "%.1f\"", limelightDistance));
                telemetry.addData("Auto Power", String.format(Locale.US, "%.0f%%", autoPowerLevel * 100));
                telemetry.addData("tx/ty", String.format(Locale.US, "%.1f° / %.1f°", limelightTx, limelightTy));
            } else {
                telemetry.addData("Target", "NOT DETECTED");
                telemetry.addLine("  (Using last known or manual)");
            }
        } else {
            // Show manual preset selection
            telemetry.addData("Selected", DISTANCE_NAMES[currentDistanceIndex] +
                    " (" + (int)DISTANCE_PRESETS[currentDistanceIndex] + "\")");
            telemetry.addData("Power Level", String.format(Locale.US, "%.0f%%", POWER_PRESETS[currentDistanceIndex] * 100));
        }

        // Distance-Power Table (compact)
        telemetry.addLine("\nDistance Table:");
        StringBuilder tableStr = new StringBuilder();
        for (int i = 0; i < DISTANCE_PRESETS.length; i++) {
            String marker = (!autoDistanceMode && i == currentDistanceIndex) ? ">" : " ";
            tableStr.append(String.format(Locale.US, "%s%.0f\":%.0f%% ",
                    marker, DISTANCE_PRESETS[i], POWER_PRESETS[i] * 100));
            if (i == 2) tableStr.append("\n");
        }
        telemetry.addLine(tableStr.toString());

        // Shooter Status
        telemetry.addLine("\n─── SHOOTER STATUS ───");
        double currentVelocity = shooter.getVelocity();
        double currentRPM = (currentVelocity / MOTOR_TICKS_PER_REV) * 60.0;
        double targetRPM = (currentTargetVelocity / MOTOR_TICKS_PER_REV) * 60.0;

        telemetry.addData("Shooter", shooterRunning ? "SPINNING" : "OFF");
        if (shooterRunning) {
            telemetry.addData("Current RPM", String.format(Locale.US, "%.0f", currentRPM));
            telemetry.addData("Target RPM", String.format(Locale.US, "%.0f", targetRPM));
            telemetry.addData("Ready to Fire", shooterVelocityReached ? "YES" : "NO");
        }

        // Mechanisms
        telemetry.addLine("\n─── MECHANISMS ───");
        telemetry.addData("Intake", frontIntake.getPower() > 0 ? "FORWARD" :
                (frontIntake.getPower() < 0 ? "REVERSE" : "OFF"));

        // Turret with lock-on and tracking info
        String turretStatus = String.format(Locale.US, "%.2f", turretPosition);
        if (autoDistanceMode) {
            if (turretLockedOn) {
                if (limelightHasTarget) {
                    turretStatus += String.format(Locale.US, " [LOCKED tx:%.1f°]", limelightTx);
                } else {
                    turretStatus += " [LOCKED - searching]";
                }
            } else {
                turretStatus += " [UNLOCKED]";
            }
        }
        telemetry.addData("Turret", turretStatus);
        telemetry.addData("Pusher", pusherServo.getPower() > 0 ? "FORWARD" : pusherServo.getPower() < 0 ? "REVERSE" : "STOPPED");

        // Limelight Info
        addLimelightTelemetry();

        // Controls Reference (compact)
        telemetry.addLine("\n─── CONTROLS ───");
        telemetry.addLine("GP1: Sticks=Drive, B=Auto-move");
        telemetry.addLine("GP2: Y=Manual, D-pad=Dist/Turret");
        telemetry.addLine("GP2: RB=Shoot, RT=Push, LB=Intake");

        telemetry.update();
    }

    /**
     * Displays Limelight telemetry data.
     * Reference: https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
     *
     * Key LLResult methods used:
     *   - isValid(): Check if result contains valid data
     *   - getStaleness(): Data age in milliseconds
     *   - getPipelineIndex(): Current active pipeline
     *   - getFiducialResults(): List of detected AprilTags
     *   - getBotpose(): MegaTag 1 robot position (field coordinates)
     *
     * Key FiducialResult methods used:
     *   - getFiducialId(): AprilTag ID number
     *   - getTargetXDegrees(): Horizontal offset (tx)
     *   - getTargetYDegrees(): Vertical offset (ty)
     *   - getRobotPoseTargetSpace(): Robot pose relative to tag
     */
    private void addLimelightTelemetry() {
        telemetry.addLine("\n─── LIMELIGHT ───");

        if (!limelightConnected) {
            telemetry.addData("Status", "Not Connected");
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addData("Status", "No Data");
            return;
        }

        // Show pipeline and data freshness
        telemetry.addData("Pipeline", result.getPipelineIndex());
        telemetry.addData("Staleness", String.format(Locale.US, "%dms", result.getStaleness()));

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        telemetry.addData("AprilTags", fiducials.size());

        // Show detected tag IDs
        if (!fiducials.isEmpty()) {
            StringBuilder tagIds = new StringBuilder();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                tagIds.append(fr.getFiducialId()).append(" ");
            }
            telemetry.addData("Tag IDs", tagIds.toString());
        }

        // Show robot pose from MegaTag 1 (getBotpose)
        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            telemetry.addData("Bot Pose", String.format(Locale.US, "X:%.1f Y:%.1f",
                    botpose.getPosition().x * 39.3701,
                    botpose.getPosition().y * 39.3701));
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // STOP
    // ═══════════════════════════════════════════════════════════════════

    @Override
    public void stop() {
        // Stop all mechanisms
        stopShooter();
        frontIntake.setPower(0);
        pusherServo.setPower(0);
        indicator.setPosition(INDICATOR_OFF);

        // Stop Limelight
        if (limelightConnected) {
            limelight.stop();
        }
    }
}
