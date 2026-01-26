package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "RedAuto", group = "Autonomous")
@Configurable // Panels
public class RedAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    // Constants from pedroPathing/Constants.java
    public FollowerConstants followerConstants = Constants.followerConstants; // Mass, zero power acceleration
    public MecanumConstants driveConstants = Constants.driveConstants; // Mecanum drive constants
    public PinpointConstants pinpointConstants = Constants.localizerConstants; // Pinpoint odometry constants
    public PathConstraints pathConstraints = Constants.pathConstraints; // Path following constraints
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Mecanum drive motors
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // Pinpoint odometry (used by Follower internally, kept for direct access if needed)
    @SuppressWarnings("unused")
    private GoBildaPinpointDriver pinpoint;

    // Limelight 3A Vision Sensor
    private Limelight3A limelight;

    // Intake motors
    private DcMotor frontIntake;
    private DcMotor rearIntake;

    // Servos
    private Servo pusherServo;
    private Servo turretGear;

    // Shooter motor (DcMotorEx for encoder velocity)
    private DcMotorEx shooter;

    // Servo positions
    private static final double PUSHER_RETRACTED = 0.2;
    private static final double PUSHER_EXTENDED = 0.5;
    private static final double TURRET_HOME = 0.5;

    // Turret aiming constants
    private static final double TURRET_CENTER = 0.5;      // Servo position when centered
    private static final double TURRET_MIN = 0.0;         // Minimum servo position
    private static final double TURRET_MAX = 1.0;         // Maximum servo position
    private static final double TURRET_RANGE_DEG = 90.0;  // Turret range in degrees (adjust for your servo)
    private static final double AIM_TOLERANCE = 2.0;    // Degrees of acceptable error

    // Limelight pipeline configuration (0-9, configure pipelines in Limelight web interface)
    private static final int LIMELIGHT_PIPELINE = 6;

    // Intake power
    private static final double INTAKE_POWER = 1.0;

    // Shooter power (fallback if no AprilTag detected)
    private static final double SHOOTER_POWER_DEFAULT = 1.0;

    // Distance-based shooter power lookup table (calibrate on field)
    // Distances in inches, powers from 0.0 to 1.0
    private static final double[] SHOOTER_DISTANCES = {24.0, 48.0, 72.0};
    private static final double[] SHOOTER_POWERS =    {0.7, 0.8, 1.0};

    // Distance estimation constants (adjust for camera mounting)
    private static final double CAMERA_HEIGHT_INCHES = 13.5;   // Camera height from ground
    private static final double TARGET_HEIGHT_INCHES = 30.0;   // AprilTag height from ground
    private static final double CAMERA_MOUNT_ANGLE_DEG = 15.0; // Camera tilt angle (positive = tilted up)

    // Current calculated values (for telemetry)
    private double currentDistance = 0;
    private double currentShooterPower = SHOOTER_POWER_DEFAULT;

    // Shooting sequence constants
    private static final int SHOTS_PER_POSITION = 3;      // Number of balls to shoot at each position
    private static final double PUSHER_CYCLE_TIME = 0.25;  // Time for pusher extend/retract cycle
    private static final double INTAKE_DELAY = 0.25;      // Delay before intake activates after shooter starts

    // Waypoint pause constant
    private static final double WAYPOINT_PAUSE = 0.125;   // Pause time at waypoints for intake activation

    // Drive speed constants
    private static final double NORMAL_SPEED = 1.0;       // Normal drive speed
    private static final double SLOW_SPEED = 0.5;         // Slow drive speed for collection waypoints

    // Timer for actions
    private double actionTimer = 0;
    private double shooterStartTime = 0;   // Time when shooter started (for intake delay)
    private double waypointPauseTimer = 0; // Timer for waypoint pauses
    private boolean turretLocked = false;  // Flag to indicate turret is locked on target
    private int shotCount = 0;             // Counter for shots fired at current position

    // AprilTag position correction (sensor fusion)
    private boolean aprilTagCorrectionEnabled = true;
    private double lastCorrectionTime = 0;
    private static final double CORRECTION_COOLDOWN = 0.5;  // Minimum seconds between corrections
    private int correctionCount = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize mecanum drive motors using names from driveConstants
        leftFront = hardwareMap.get(DcMotor.class, driveConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotor.class, driveConstants.leftRearMotorName);
        rightFront = hardwareMap.get(DcMotor.class, driveConstants.rightFrontMotorName);
        rightRear = hardwareMap.get(DcMotor.class, driveConstants.rightRearMotorName);

        // Set motor directions from driveConstants
        leftFront.setDirection(driveConstants.leftFrontMotorDirection);
        leftRear.setDirection(driveConstants.leftRearMotorDirection);
        rightFront.setDirection(driveConstants.rightFrontMotorDirection);
        rightRear.setDirection(driveConstants.rightRearMotorDirection);

        // Set zero power behavior to brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Pinpoint odometry for direct access (Follower also uses it internally)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointConstants.hardwareMapName);

        // Initialize intake motors
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        rearIntake = hardwareMap.get(DcMotor.class, "rearIntake");
        frontIntake.setDirection(DcMotor.Direction.FORWARD);
        rearIntake.setDirection(DcMotor.Direction.REVERSE);

        // Initialize shooter motor with encoder
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize servos
        pusherServo = hardwareMap.get(Servo.class, "pusherServo");
        turretGear = hardwareMap.get(Servo.class, "turretGear");
        pusherServo.setPosition(PUSHER_RETRACTED);
        turretGear.setPosition(TURRET_HOME);

        // Initialize Limelight 3A Vision Sensor
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);  // Use configured pipeline
        limelight.start();  // Start polling for data

        // Create Follower - this handles Pinpoint configuration via Constants
        follower = Constants.createFollower(hardwareMap);
        // Starting pose matches Shoot1's start position
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing

        // Apply AprilTag position correction if enabled (sensor fusion)
        if (aprilTagCorrectionEnabled) {
            applyAprilTagCorrection();
        }

        // Continuously track AprilTag with turret throughout autonomous
        aimTurretAtAprilTag();

        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        // AprilTag and turret telemetry
        panelsTelemetry.debug("AprilTag Detected", hasAprilTag());
        if (hasAprilTag()) {
            panelsTelemetry.debug("AprilTag ID", getFirstAprilTagId());
            panelsTelemetry.debug("AprilTag TX", getAprilTagTx());
            panelsTelemetry.debug("AprilTag TY", getAprilTagTy());
            panelsTelemetry.debug("Turret Aim Pos", getTurretAimPosition());
        }
        panelsTelemetry.debug("Turret Locked", isTurretLocked());
        panelsTelemetry.debug("Shot Count", shotCount + "/" + SHOTS_PER_POSITION);

        // Shooter telemetry
        panelsTelemetry.debug("Shooter Power", shooter.getPower());
        panelsTelemetry.debug("Shooter Velocity", shooter.getVelocity());
        panelsTelemetry.debug("Target Distance", String.format("%.1f in", currentDistance));
        panelsTelemetry.debug("Calculated Power", String.format("%.2f", currentShooterPower));

        // AprilTag correction telemetry
        panelsTelemetry.debug("AT Correction", aprilTagCorrectionEnabled ? "ON" : "OFF");
        panelsTelemetry.debug("Corrections", correctionCount);
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Shoot1;
        public PathChain Waypoint1;
        public PathChain Waypoint2;
        public PathChain Waypoint3;
        public PathChain Shoot2;
        public PathChain Waypoint4;
        public PathChain Waypoint5;
        public PathChain Waypoint6;
        public PathChain Shoot3;
        public PathChain Waypoint7;
        public PathChain Waypoint8;
        public PathChain Shoot4End;

        public Paths(Follower follower) {
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),

                                    new Pose(77.48072487644151, 88.2504118616145)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                    .build();

            Waypoint1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(77.48072487644151, 88.2504118616145),

                                    new Pose(90.38550247116966, 35.82207578253708)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Waypoint2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.38550247116966, 35.82207578253708),

                                    new Pose(113.39703459637556, 35.34761120263588)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Waypoint3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(113.39703459637556, 35.34761120263588),

                                    new Pose(90.38550247116967, 35.82207578253709)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.38550247116967, 35.82207578253709),

                                    new Pose(77.48072487644151, 88.2504118616145)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                    .build();

            Waypoint4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(77.48072487644151, 88.2504118616145),

                                    new Pose(89.13838550247117, 59.30807248764413)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Waypoint5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.13838550247117, 59.30807248764413),

                                    new Pose(113.34266886326193, 59.54530477759474)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Waypoint6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(113.34266886326193, 59.54530477759474),

                                    new Pose(88.97841845140034, 59.070840197693556)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.97841845140034, 59.070840197693556),

                                    new Pose(84.04876441515651, 96.31630971993411)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                    .build();

            Waypoint7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.04876441515651, 96.31630971993411),

                                    new Pose(96.25601317957168, 83.74299835255357)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            Waypoint8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.25601317957168, 83.74299835255357),

                                    new Pose(112.10543657331135, 83.2685337726524)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shoot4End = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(112.10543657331135, 83.2685337726524),

                                    new Pose(83.98023064250411, 96.3163097199341)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            // ========== SHOOT 1 ==========
            case 0: // Start - follow path to Shoot1, start shooter early
                follower.followPath(paths.Shoot1, true);
                setShooterPower(getDistanceAdjustedShooterPower());      // Start shooter early for spin-up
                shooterStartTime = getRuntime();     // Record for intake delay
                setIntakePower(0);
                pathState = 1;
                break;

            case 1: // Wait for Shoot1 path to complete
                if (!follower.isBusy()) {
                    // Shooter already running from state 0
                    shotCount = 0;                    // Reset shot counter
                    actionTimer = getRuntime();
                    pathState = 2;
                }
                break;

            case 2: // Wait for turret lock, then fire
                // Activate intake after delay
                if (getRuntime() - shooterStartTime > INTAKE_DELAY) {
                    setIntakePower(INTAKE_POWER);
                }
                if (isTurretLocked() || (getRuntime() - actionTimer > 1.5)) {
                    pusherServo.setPosition(PUSHER_EXTENDED);
                    shotCount++;
                    actionTimer = getRuntime();
                    pathState = 3;
                }
                break;

            case 3: // Wait for pusher cycle, check if more shots needed
                if (getRuntime() - actionTimer > PUSHER_CYCLE_TIME) {
                    pusherServo.setPosition(PUSHER_RETRACTED);
                    if (shotCount < SHOTS_PER_POSITION) {
                        // More shots needed - wait briefly then shoot again
                        actionTimer = getRuntime();
                        pathState = 2;
                    } else {
                        // All shots fired - move to next waypoint
                        setShooterPower(0);
                        setIntakePower(0);  // Turn off intake at end of shooter cycle
                        follower.followPath(paths.Waypoint1, true);
                        setIntakePower(INTAKE_POWER);  // Turn on for collection during travel
                        pathState = 4;
                    }
                }
                break;

            // ========== WAYPOINTS 1-3 ==========
            case 4: // Wait for Waypoint1, then Waypoint2 (slow speed)
                if (!follower.isBusy()) {
                    follower.setMaxPower(SLOW_SPEED);  // Slow down for collection
                    follower.followPath(paths.Waypoint2, true);
                    pathState = 5;
                }
                break;

            case 5: // Wait for Waypoint2, pause for intake, then Waypoint3
                if (!follower.isBusy()) {
                    if (waypointPauseTimer == 0) {
                        waypointPauseTimer = getRuntime();  // Start pause timer
                    } else if (getRuntime() - waypointPauseTimer > WAYPOINT_PAUSE) {
                        follower.setMaxPower(NORMAL_SPEED);  // Reset to normal speed
                        follower.followPath(paths.Waypoint3, true);
                        waypointPauseTimer = 0;  // Reset for next use
                        pathState = 6;
                    }
                }
                break;

            case 6: // Wait for Waypoint3, then Shoot2, start shooter early
                if (!follower.isBusy()) {
                    setIntakePower(0);
                    follower.followPath(paths.Shoot2, true);
                    setShooterPower(getDistanceAdjustedShooterPower());      // Start shooter early for spin-up
                    shooterStartTime = getRuntime();     // Record for intake delay
                    pathState = 7;
                }
                break;

            // ========== SHOOT 2 ==========
            case 7: // Wait for Shoot2 path to complete
                if (!follower.isBusy()) {
                    // Shooter already running from state 6
                    shotCount = 0;                    // Reset shot counter
                    actionTimer = getRuntime();
                    pathState = 8;
                }
                break;

            case 8: // Wait for turret lock, then fire
                // Activate intake after delay
                if (getRuntime() - shooterStartTime > INTAKE_DELAY) {
                    setIntakePower(INTAKE_POWER);
                }
                if (isTurretLocked() || (getRuntime() - actionTimer > 1.5)) {
                    pusherServo.setPosition(PUSHER_EXTENDED);
                    shotCount++;
                    actionTimer = getRuntime();
                    pathState = 9;
                }
                break;

            case 9: // Wait for pusher cycle, check if more shots needed
                if (getRuntime() - actionTimer > PUSHER_CYCLE_TIME) {
                    pusherServo.setPosition(PUSHER_RETRACTED);
                    if (shotCount < SHOTS_PER_POSITION) {
                        // More shots needed - wait briefly then shoot again
                        actionTimer = getRuntime();
                        pathState = 8;
                    } else {
                        // All shots fired - move to next waypoint
                        setShooterPower(0);
                        setIntakePower(0);  // Turn off intake at end of shooter cycle
                        follower.followPath(paths.Waypoint4, true);
                        setIntakePower(INTAKE_POWER);  // Turn on for collection during travel
                        pathState = 10;
                    }
                }
                break;

            // ========== WAYPOINTS 4-6 ==========
            case 10: // Wait for Waypoint4, pause for intake, then Waypoint5 (slow speed)
                if (!follower.isBusy()) {
                    if (waypointPauseTimer == 0) {
                        waypointPauseTimer = getRuntime();  // Start pause timer
                    } else if (getRuntime() - waypointPauseTimer > WAYPOINT_PAUSE) {
                        follower.setMaxPower(SLOW_SPEED);  // Slow down for collection
                        follower.followPath(paths.Waypoint5, true);
                        waypointPauseTimer = 0;  // Reset for next use
                        pathState = 11;
                    }
                }
                break;

            case 11: // Wait for Waypoint5, then Waypoint6
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_SPEED);  // Reset to normal speed
                    follower.followPath(paths.Waypoint6, true);
                    pathState = 12;
                }
                break;

            case 12: // Wait for Waypoint6, then Shoot3, start shooter early
                if (!follower.isBusy()) {
                    setIntakePower(0);
                    follower.followPath(paths.Shoot3, true);
                    setShooterPower(getDistanceAdjustedShooterPower());      // Start shooter early for spin-up
                    shooterStartTime = getRuntime();     // Record for intake delay
                    pathState = 13;
                }
                break;

            // ========== SHOOT 3 ==========
            case 13: // Wait for Shoot3 path to complete
                if (!follower.isBusy()) {
                    // Shooter already running from state 12
                    shotCount = 0;                    // Reset shot counter
                    actionTimer = getRuntime();
                    pathState = 14;
                }
                break;

            case 14: // Wait for turret lock, then fire
                // Activate intake after delay
                if (getRuntime() - shooterStartTime > INTAKE_DELAY) {
                    setIntakePower(INTAKE_POWER);
                }
                if (isTurretLocked() || (getRuntime() - actionTimer > 1.5)) {
                    pusherServo.setPosition(PUSHER_EXTENDED);
                    shotCount++;
                    actionTimer = getRuntime();
                    pathState = 15;
                }
                break;

            case 15: // Wait for pusher cycle, check if more shots needed
                if (getRuntime() - actionTimer > PUSHER_CYCLE_TIME) {
                    pusherServo.setPosition(PUSHER_RETRACTED);
                    if (shotCount < SHOTS_PER_POSITION) {
                        // More shots needed - wait briefly then shoot again
                        actionTimer = getRuntime();
                        pathState = 14;
                    } else {
                        // All shots fired - move to next waypoint
                        setShooterPower(0);
                        setIntakePower(0);  // Turn off intake at end of shooter cycle
                        follower.followPath(paths.Waypoint7, true);
                        setIntakePower(INTAKE_POWER);  // Turn on for collection during travel
                        pathState = 16;
                    }
                }
                break;

            // ========== WAYPOINTS 7-8 ==========
            case 16: // Wait for Waypoint7, pause for intake, then Waypoint8 (slow speed)
                if (!follower.isBusy()) {
                    if (waypointPauseTimer == 0) {
                        waypointPauseTimer = getRuntime();  // Start pause timer
                    } else if (getRuntime() - waypointPauseTimer > WAYPOINT_PAUSE) {
                        follower.setMaxPower(SLOW_SPEED);  // Slow down for collection
                        follower.followPath(paths.Waypoint8, true);
                        waypointPauseTimer = 0;  // Reset for next use
                        pathState = 17;
                    }
                }
                break;

            case 17: // Wait for Waypoint8, then Shoot4End, start shooter early
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_SPEED);  // Reset to normal speed
                    setIntakePower(0);
                    follower.followPath(paths.Shoot4End, true);
                    setShooterPower(getDistanceAdjustedShooterPower());      // Start shooter early for spin-up
                    shooterStartTime = getRuntime();     // Record for intake delay
                    pathState = 18;
                }
                break;

            // ========== SHOOT 4 (FINAL) ==========
            case 18: // Wait for Shoot4End path to complete
                if (!follower.isBusy()) {
                    // Shooter already running from state 17
                    shotCount = 0;                    // Reset shot counter
                    actionTimer = getRuntime();
                    pathState = 19;
                }
                break;

            case 19: // Wait for turret lock, then fire
                // Activate intake after delay
                if (getRuntime() - shooterStartTime > INTAKE_DELAY) {
                    setIntakePower(INTAKE_POWER);
                }
                if (isTurretLocked() || (getRuntime() - actionTimer > 1.5)) {
                    pusherServo.setPosition(PUSHER_EXTENDED);
                    shotCount++;
                    actionTimer = getRuntime();
                    pathState = 20;
                }
                break;

            case 20: // Wait for pusher cycle, check if more shots needed
                if (getRuntime() - actionTimer > PUSHER_CYCLE_TIME) {
                    pusherServo.setPosition(PUSHER_RETRACTED);
                    if (shotCount < SHOTS_PER_POSITION) {
                        // More shots needed - wait briefly then shoot again
                        actionTimer = getRuntime();
                        pathState = 19;
                    } else {
                        // All shots fired - autonomous complete
                        setShooterPower(0);
                        setIntakePower(0);  // Turn off intake at end of shooter cycle
                        pathState = 21;
                    }
                }
                break;

            // ========== DONE ==========
            case 21: // Autonomous complete
                setIntakePower(0);
                setShooterPower(0);
                stopDrive();
                break;

            default:
                // Invalid state - stop everything for safety
                setIntakePower(0);
                setShooterPower(0);
                stopDrive();
                break;
        }
        return pathState;
    }

    /**
     * Set intake motor power.
     * @param power Power level (-1.0 to 1.0)
     */
    private void setIntakePower(double power) {
        frontIntake.setPower(power);
        rearIntake.setPower(power);
    }

    /**
     * Set shooter motor power.
     * @param power Power level (-1.0 to 1.0)
     */
    private void setShooterPower(double power) {
        shooter.setPower(power);
    }

    /**
     * Get the distance-adjusted shooter power based on AprilTag detection.
     * Updates currentDistance and currentShooterPower for telemetry.
     *
     * @return calculated power based on distance, or default if no target
     */
    private double getDistanceAdjustedShooterPower() {
        if (!hasAprilTag()) {
            currentDistance = 0;
            currentShooterPower = SHOOTER_POWER_DEFAULT;
            return SHOOTER_POWER_DEFAULT;
        }

        // Estimate distance from AprilTag
        currentDistance = estimateDistanceFromAprilTag();

        // Interpolate power from lookup table
        currentShooterPower = interpolateShooterPower(currentDistance);

        return currentShooterPower;
    }

    /**
     * Estimate distance to AprilTag using ty angle or 3D pose.
     *
     * @return estimated distance in inches
     */
    private double estimateDistanceFromAprilTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0;
        }

        // Try to get distance from AprilTag 3D pose first (more accurate)
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (!fiducials.isEmpty()) {
            LLResultTypes.FiducialResult tag = fiducials.get(0);
            Pose3D cameraPose = tag.getCameraPoseTargetSpace();
            if (cameraPose != null) {
                double x = cameraPose.getPosition().x;
                double y = cameraPose.getPosition().y;
                double z = cameraPose.getPosition().z;
                // Convert meters to inches (Limelight returns meters)
                return Math.sqrt(x * x + y * y + z * z) * 39.3701;
            }
        }

        // Fallback: estimate from ty angle
        double ty = result.getTy();
        double angleToTargetRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + ty);
        double heightDiff = TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;

        if (Math.abs(angleToTargetRad) < 0.01) {
            return 0;
        }

        return Math.abs(heightDiff / Math.tan(angleToTargetRad));
    }

    /**
     * Interpolate shooter power from lookup table based on distance.
     *
     * @param distance distance to target in inches
     * @return interpolated power value
     */
    private double interpolateShooterPower(double distance) {
        // Handle edge cases
        if (distance <= SHOOTER_DISTANCES[0]) {
            return SHOOTER_POWERS[0];
        }
        if (distance >= SHOOTER_DISTANCES[SHOOTER_DISTANCES.length - 1]) {
            return SHOOTER_POWERS[SHOOTER_POWERS.length - 1];
        }

        // Find the two points to interpolate between
        for (int i = 0; i < SHOOTER_DISTANCES.length - 1; i++) {
            if (distance >= SHOOTER_DISTANCES[i] && distance <= SHOOTER_DISTANCES[i + 1]) {
                // Linear interpolation
                double t = (distance - SHOOTER_DISTANCES[i]) /
                           (SHOOTER_DISTANCES[i + 1] - SHOOTER_DISTANCES[i]);
                return SHOOTER_POWERS[i] + t * (SHOOTER_POWERS[i + 1] - SHOOTER_POWERS[i]);
            }
        }

        return SHOOTER_POWER_DEFAULT;
    }

    /**
     * Drive the robot using mecanum drive kinematics.
     * @param forward Forward/backward power (-1 to 1, positive = forward)
     * @param strafe Left/right power (-1 to 1, positive = right)
     * @param rotate Rotation power (-1 to 1, positive = clockwise)
     */
    @SuppressWarnings("unused")
    public void mecanumDrive(double forward, double strafe, double rotate) {
        double maxPower = driveConstants.maxPower;

        // Calculate motor powers using mecanum drive kinematics
        double leftFrontPower = forward + strafe + rotate;
        double leftRearPower = forward - strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double rightRearPower = forward + strafe - rotate;

        // Normalize powers if any exceed maxPower
        double maxCalculated = Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(leftRearPower),
                        Math.max(Math.abs(rightFrontPower), Math.abs(rightRearPower))));
        if (maxCalculated > maxPower) {
            leftFrontPower /= maxCalculated / maxPower;
            leftRearPower /= maxCalculated / maxPower;
            rightFrontPower /= maxCalculated / maxPower;
            rightRearPower /= maxCalculated / maxPower;
        }

        // Set motor powers
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    /**
     * Stop all drive motors (direct motor control).
     */
    public void stopDrive() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * Drive using Follower's teleop drive (recommended method from Tuning.java).
     * Call follower.startTeleopDrive() before using this in your state machine.
     * @param forward Forward/backward power (-1 to 1, positive = forward)
     * @param strafe Left/right power (-1 to 1, positive = left)
     * @param rotate Rotation power (-1 to 1, positive = counterclockwise)
     * @param robotCentric true for robot-centric, false for field-centric
     */
    @SuppressWarnings("unused")
    public void followerDrive(double forward, double strafe, double rotate, boolean robotCentric) {
        follower.setTeleOpDrive(forward, strafe, rotate, robotCentric);
    }

    /**
     * Stop the robot using Follower's teleop drive (recommended method from Tuning.java).
     */
    @SuppressWarnings("unused")
    public void followerStop() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    // ======================== LIMELIGHT METHODS ========================

    /**
     * Check if the Limelight has a valid target.
     *
     * @return true if a valid result is available
     */
    public boolean hasAprilTag() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && !result.getFiducialResults().isEmpty();
    }

    /**
     * Get the horizontal offset to target (tx) in degrees.
     *
     * @return tx value, or 0 if no valid target
     */
    public double getAprilTagTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }

    /**
     * Get the vertical offset to target (ty) in degrees.
     *
     * @return ty value, or 0 if no valid target
     */
    public double getAprilTagTy() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0;
    }

    /**
     * Get the robot's field position from AprilTag detection (botpose).
     *
     * @return Pose3D of robot position, or null if not available
     */
    @SuppressWarnings("unused")
    public Pose3D getBotpose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    /**
     * Apply AprilTag position correction to Pinpoint odometry (sensor fusion).
     * Uses the robot's field position from AprilTag detection to correct odometry drift.
     * Has a cooldown to prevent over-correction.
     */
    private void applyAprilTagCorrection() {
        // Only correct if we have a valid AprilTag and cooldown has passed
        if (!hasAprilTag()) {
            return;
        }

        if (getRuntime() - lastCorrectionTime < CORRECTION_COOLDOWN) {
            return;
        }

        // Get botpose from Limelight (robot's field position from AprilTag)
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            return;
        }

        // Convert botpose to inches (Limelight returns meters)
        double aprilX = botpose.getPosition().x * 39.3701;
        double aprilY = botpose.getPosition().y * 39.3701;
        double aprilHeading = Math.toDegrees(botpose.getOrientation().getYaw());

        // Update Pinpoint position with AprilTag-derived position
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, aprilX, aprilY,
                AngleUnit.DEGREES, aprilHeading));

        // Also update Follower's pose to keep path following in sync
        follower.setPose(new Pose(aprilX, aprilY, Math.toRadians(aprilHeading)));

        lastCorrectionTime = getRuntime();
        correctionCount++;
    }

    /**
     * Get detected AprilTag/fiducial results.
     *
     * @return List of fiducial results
     */
    public List<LLResultTypes.FiducialResult> getAprilTagResults() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return java.util.Collections.emptyList();
    }

    /**
     * Get the ID of the first detected AprilTag.
     *
     * @return AprilTag ID, or -1 if none detected
     */
    public int getFirstAprilTagId() {
        List<LLResultTypes.FiducialResult> tags = getAprilTagResults();
        if (!tags.isEmpty()) {
            return tags.get(0).getFiducialId();
        }
        return -1;
    }

    /**
     * Switch the Limelight pipeline.
     *
     * @param pipelineIndex pipeline number (0-9)
     */
    @SuppressWarnings("unused")
    public void setLimelightPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    // ======================== TURRET AIMING METHODS ========================

    /**
     * Aim the turret at the detected AprilTag using Limelight data.
     * Converts the tx (horizontal offset) to a servo position.
     *
     * @return true if turret is aimed within tolerance, false otherwise
     */
    public boolean aimTurretAtAprilTag() {
        if (!hasAprilTag()) {
            // No target - hold current position
            turretLocked = false;
            return false;
        }

        double tx = getAprilTagTx();

        // Check if we're within tolerance
        if (Math.abs(tx) < AIM_TOLERANCE) {
            turretLocked = true;
            return true;
        }

        // Convert tx (degrees) to servo position adjustment
        // tx is positive when target is right, negative when left
        // Negate to move servo toward target (servo direction is inverted)
        double servoAdjustment = -(tx / TURRET_RANGE_DEG);
        double targetPosition = TURRET_CENTER + servoAdjustment;

        // Clamp to valid servo range
        targetPosition = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetPosition));

        turretGear.setPosition(targetPosition);
        turretLocked = false;
        return false;
    }

    /**
     * Get the current turret servo position needed to aim at the AprilTag.
     *
     * @return servo position (0.0 to 1.0), or TURRET_CENTER if no target
     */
    public double getTurretAimPosition() {
        if (!hasAprilTag()) {
            return TURRET_CENTER;
        }

        double tx = getAprilTagTx();
        double servoAdjustment = -(tx / TURRET_RANGE_DEG);
        double targetPosition = TURRET_CENTER + servoAdjustment;

        return Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetPosition));
    }

    /**
     * Check if the turret is locked on target (within tolerance).
     *
     * @return true if aimed within tolerance
     */
    public boolean isTurretLocked() {
        return turretLocked && hasAprilTag() && Math.abs(getAprilTagTx()) < AIM_TOLERANCE;
    }

    @Override
    public void stop() {
        // Stop all hardware when OpMode stops
        stopDrive();
        setIntakePower(0);
        setShooterPower(0);
        pusherServo.setPosition(PUSHER_RETRACTED);
        turretGear.setPosition(TURRET_HOME);
        limelight.stop();
    }
}
