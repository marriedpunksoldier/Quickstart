package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.List;

/**
 * DecodeRed - Full-featured TeleOp with AprilTag tracking, Pinpoint odometry, and variable shooter speed
 *
 * Features:
 *   - GoBILDA Pinpoint odometry for continuous position tracking
 *   - AprilTag-based position correction (sensor fusion)
 *   - Distance-based automatic shooter power adjustment
 *   - Auto-aim turret tracking
 *
 * GAMEPAD 1 (Driver):
 *   Left Stick: Drive (forward/strafe)
 *   Right Stick X: Rotate
 *   Right Trigger: Intake IN
 *   Left Trigger: Intake OUT (reverse)
 *   Left Bumper: Slow mode (50% speed)
 *   X: Toggle AprilTag position correction
 *   B: Reset odometry to origin
 *
 * GAMEPAD 2 (Operator):
 *   Right Trigger: Spin up shooter (variable power based on distance)
 *   Left Trigger: Spin up shooter (manual preset power)
 *   Left Bumper: Fire (pusher)
 *   Right Bumper: Toggle auto-aim
 *   D-Pad Left/Right: Manual turret control
 *   D-Pad Up/Down: Cycle shooter power presets
 *   A: Toggle auto-power mode
 *   Y: Reset turret to center
 *   B: Emergency stop shooter
 */
@TeleOp(name = "DecodeRed", group = "TeleOp")
public class DecodeRed extends OpMode {

    // Constants from Pedro Pathing
    private MecanumConstants driveConstants = Constants.driveConstants;

    // GoBILDA Pinpoint Odometry
    private GoBildaPinpointDriver pinpoint;

    // Mecanum drive motors
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;

    // Intake motors
    private DcMotor frontIntake;
    private DcMotor rearIntake;

    // Shooter motor (DcMotorEx for encoder velocity)
    private DcMotorEx shooter;

    // Servos
    private Servo pusherServo;
    private Servo turretGear;

    // Limelight 3A Vision Sensor
    private Limelight3A limelight;

    // Servo positions
    private static final double PUSHER_RETRACTED = 0.2;
    private static final double PUSHER_EXTENDED = 0.5;
    private static final double TURRET_HOME = 0.5;

    // Turret aiming constants
    private static final double TURRET_CENTER = 0.5;
    private static final double TURRET_MIN = 0.0;
    private static final double TURRET_MAX = 1.0;
    private static final double TURRET_RANGE_DEG = 90.0;
    private static final double AIM_TOLERANCE = 2.0;

    // Limelight pipeline
    private static final int LIMELIGHT_PIPELINE = 6;

    // Drive constants
    private static final double SLOW_MODE_MULTIPLIER = 0.5;

    // Intake power
    private static final double INTAKE_POWER = 1.0;

    // Shooter power presets (for manual mode)
    private static final double[] SHOOTER_PRESETS = {0.7, 0.75, 0.8, 0.85, 1.0};
    private int presetIndex = 2;  // Start at 0.8

    // Shooter power (fallback if no AprilTag detected)
    private static final double SHOOTER_POWER_DEFAULT = 1.0;

    // Distance-based shooter power lookup table (calibrate on field)
    private static final double[] SHOOTER_DISTANCES = {24.0, 48.0, 72.0};
    private static final double[] SHOOTER_POWERS =    {0.7, 0.8, 1.0};

    // Distance estimation constants
    private static final double CAMERA_HEIGHT_INCHES = 13.5;
    private static final double TARGET_HEIGHT_INCHES = 30.0;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 15.0;

    // State variables
    private double turretPosition = TURRET_CENTER;
    private boolean turretLocked = false;
    private boolean autoAimEnabled = true;
    private boolean autoPowerEnabled = true;
    private double currentDistance = 0;
    private double currentShooterPower = SHOOTER_POWER_DEFAULT;

    // Firing state
    private boolean isFiring = false;
    private double fireStartTime = 0;
    private static final double FIRE_DURATION = 0.3;

    // Button edge detection
    private boolean lastRightBumper2 = false;
    private boolean lastAButton2 = false;
    private boolean lastDpadUp2 = false;
    private boolean lastDpadDown2 = false;
    private boolean lastXButton1 = false;
    private boolean lastBButton1 = false;

    // Pinpoint odometry state
    private double robotX = 0;      // Robot X position (inches)
    private double robotY = 0;      // Robot Y position (inches)
    private double robotHeading = 0; // Robot heading (degrees)
    private double robotVelocityX = 0;
    private double robotVelocityY = 0;

    // AprilTag position correction
    private boolean aprilTagCorrectionEnabled = true;
    private double lastCorrectionTime = 0;
    private static final double CORRECTION_COOLDOWN = 0.5;  // Minimum seconds between corrections
    private int correctionCount = 0;

    // Known target position for distance calculation (field coordinates)
    // Set this to your target AprilTag's field position
    private static final double TARGET_X = 72.0;  // inches
    private static final double TARGET_Y = 0.0;   // inches

    @Override
    public void init() {
        // Initialize mecanum drive motors
        leftFront = hardwareMap.get(DcMotor.class, driveConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotor.class, driveConstants.leftRearMotorName);
        rightFront = hardwareMap.get(DcMotor.class, driveConstants.rightFrontMotorName);
        rightRear = hardwareMap.get(DcMotor.class, driveConstants.rightRearMotorName);

        // Set motor directions
        leftFront.setDirection(driveConstants.leftFrontMotorDirection);
        leftRear.setDirection(driveConstants.leftRearMotorDirection);
        rightFront.setDirection(driveConstants.rightFrontMotorDirection);
        rightRear.setDirection(driveConstants.rightRearMotorDirection);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        // Initialize Pinpoint odometry
        // Offsets match Constants.java: strafePodX=-1, forwardPodY=-8
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-1.0, -8.0, DistanceUnit.INCH);  // (xOffset=strafePodX, yOffset=forwardPodY)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                       GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update Pinpoint odometry
        updatePinpoint();

        // Apply AprilTag position correction if enabled
        if (aprilTagCorrectionEnabled) {
            applyAprilTagCorrection();
        }

        // Handle all controls
        handleDriveControls();
        handleIntakeControls();
        handleShooterControls();
        handleTurretControls();
        handlePusherControls();
        handleOdometryControls();

        // Update telemetry
        updateTelemetry();
    }

    // ======================== DRIVE CONTROLS ========================

    private void handleDriveControls() {
        // Get joystick values
        double forward = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;     // Left/right strafe
        double rotate = gamepad1.right_stick_x;    // Rotation

        // Apply deadzone
        forward = applyDeadzone(forward, 0.05);
        strafe = applyDeadzone(strafe, 0.05);
        rotate = applyDeadzone(rotate, 0.05);

        // Apply slow mode if left bumper held
        double speedMultiplier = gamepad1.left_bumper ? SLOW_MODE_MULTIPLIER : 1.0;
        forward *= speedMultiplier;
        strafe *= speedMultiplier;
        rotate *= speedMultiplier;

        // Calculate mecanum drive powers
        double leftFrontPower = forward + strafe + rotate;
        double leftRearPower = forward - strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double rightRearPower = forward + strafe - rotate;

        // Normalize powers
        double maxPower = Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(leftRearPower),
                        Math.max(Math.abs(rightFrontPower), Math.abs(rightRearPower))));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftRearPower /= maxPower;
            rightFrontPower /= maxPower;
            rightRearPower /= maxPower;
        }

        // Set motor powers
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) < deadzone ? 0 : value;
    }

    // ======================== INTAKE CONTROLS ========================

    private void handleIntakeControls() {
        double intakePower = 0;

        // Right trigger = intake in, Left trigger = intake out
        if (gamepad1.right_trigger > 0.1) {
            intakePower = gamepad1.right_trigger * INTAKE_POWER;
        } else if (gamepad1.left_trigger > 0.1) {
            intakePower = -gamepad1.left_trigger * INTAKE_POWER;
        }

        frontIntake.setPower(intakePower);
        rearIntake.setPower(intakePower);
    }

    // ======================== SHOOTER CONTROLS ========================

    private void handleShooterControls() {
        // Toggle auto-power mode with A button
        if (gamepad2.a && !lastAButton2) {
            autoPowerEnabled = !autoPowerEnabled;
        }
        lastAButton2 = gamepad2.a;

        // Cycle presets with D-pad up/down
        if (gamepad2.dpad_up && !lastDpadUp2) {
            presetIndex = Math.min(presetIndex + 1, SHOOTER_PRESETS.length - 1);
        }
        lastDpadUp2 = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown2) {
            presetIndex = Math.max(presetIndex - 1, 0);
        }
        lastDpadDown2 = gamepad2.dpad_down;

        // Emergency stop with B button
        if (gamepad2.b) {
            shooter.setPower(0);
            return;
        }

        // Determine shooter power
        double shooterPower = 0;

        if (gamepad2.right_trigger > 0.1) {
            // Right trigger: use auto power (distance-based) or preset if no target
            if (autoPowerEnabled) {
                shooterPower = getDistanceAdjustedShooterPower();
            } else {
                shooterPower = SHOOTER_PRESETS[presetIndex];
            }
        } else if (gamepad2.left_trigger > 0.1) {
            // Left trigger: always use manual preset
            shooterPower = SHOOTER_PRESETS[presetIndex];
        }

        shooter.setPower(shooterPower);
    }

    // ======================== TURRET CONTROLS ========================

    private void handleTurretControls() {
        // Toggle auto-aim with right bumper
        if (gamepad2.right_bumper && !lastRightBumper2) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastRightBumper2 = gamepad2.right_bumper;

        // Y button resets turret to center
        if (gamepad2.y) {
            turretPosition = TURRET_CENTER;
            autoAimEnabled = false;
            turretLocked = false;
        }

        if (autoAimEnabled && hasAprilTag()) {
            // Auto-aim at AprilTag
            aimTurretAtAprilTag();
        } else {
            // Manual turret control with D-pad left/right
            if (gamepad2.dpad_left) {
                turretPosition -= 0.01;
            } else if (gamepad2.dpad_right) {
                turretPosition += 0.01;
            }
            turretLocked = false;
        }

        // Clamp and apply turret position
        turretPosition = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretPosition));
        turretGear.setPosition(turretPosition);
    }

    private void aimTurretAtAprilTag() {
        double tx = getAprilTagTx();

        if (Math.abs(tx) < AIM_TOLERANCE) {
            turretLocked = true;
        } else {
            double servoAdjustment = -(tx / TURRET_RANGE_DEG);
            turretPosition = TURRET_CENTER + servoAdjustment;
            turretLocked = false;
        }
    }

    // ======================== PUSHER CONTROLS ========================

    private void handlePusherControls() {
        // Left bumper fires (extends pusher momentarily)
        if (gamepad2.left_bumper && !isFiring) {
            isFiring = true;
            fireStartTime = getRuntime();
            pusherServo.setPosition(PUSHER_EXTENDED);
        }

        // Auto-retract pusher after fire duration
        if (isFiring && (getRuntime() - fireStartTime) > FIRE_DURATION) {
            pusherServo.setPosition(PUSHER_RETRACTED);
            isFiring = false;
        }
    }

    // ======================== LIMELIGHT METHODS ========================

    private boolean hasAprilTag() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && !result.getFiducialResults().isEmpty();
    }

    private double getAprilTagTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }

    private double getAprilTagTy() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0;
    }

    private int getFirstAprilTagId() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (!tags.isEmpty()) {
                return tags.get(0).getFiducialId();
            }
        }
        return -1;
    }

    // ======================== PINPOINT ODOMETRY ========================

    private void updatePinpoint() {
        // Update Pinpoint to get latest position
        pinpoint.update();

        // Get current position
        Pose2D pose = pinpoint.getPosition();
        robotX = pose.getX(DistanceUnit.INCH);
        robotY = pose.getY(DistanceUnit.INCH);
        robotHeading = pose.getHeading(AngleUnit.DEGREES);

        // Get velocities
        robotVelocityX = pinpoint.getVelX(DistanceUnit.INCH);
        robotVelocityY = pinpoint.getVelY(DistanceUnit.INCH);
    }

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

        // Set Pinpoint position to AprilTag-derived position
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, aprilX, aprilY,
                AngleUnit.DEGREES, aprilHeading));

        lastCorrectionTime = getRuntime();
        correctionCount++;
    }

    private void handleOdometryControls() {
        // X button (GP1): Toggle AprilTag correction
        if (gamepad1.x && !lastXButton1) {
            aprilTagCorrectionEnabled = !aprilTagCorrectionEnabled;
        }
        lastXButton1 = gamepad1.x;

        // B button (GP1): Reset odometry to origin
        if (gamepad1.b && !lastBButton1) {
            pinpoint.resetPosAndIMU();
            correctionCount = 0;
        }
        lastBButton1 = gamepad1.b;
    }

    /**
     * Get distance to target using Pinpoint odometry position.
     * Falls back to AprilTag-based distance if no odometry available.
     */
    private double getDistanceToTargetFromOdometry() {
        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ======================== DISTANCE-BASED POWER ========================

    private double getDistanceAdjustedShooterPower() {
        if (hasAprilTag()) {
            // Primary: use AprilTag-based distance estimation
            currentDistance = estimateDistanceFromAprilTag();
        } else if (robotX != 0 || robotY != 0) {
            // Fallback: use odometry-based distance to known target
            currentDistance = getDistanceToTargetFromOdometry();
        } else {
            // No data available: use preset
            currentDistance = 0;
            currentShooterPower = SHOOTER_PRESETS[presetIndex];
            return currentShooterPower;
        }

        currentShooterPower = interpolateShooterPower(currentDistance);
        return currentShooterPower;
    }

    private double estimateDistanceFromAprilTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0;
        }

        // Try 3D pose first
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (!fiducials.isEmpty()) {
            LLResultTypes.FiducialResult tag = fiducials.get(0);
            Pose3D cameraPose = tag.getCameraPoseTargetSpace();
            if (cameraPose != null) {
                double x = cameraPose.getPosition().x;
                double y = cameraPose.getPosition().y;
                double z = cameraPose.getPosition().z;
                return Math.sqrt(x * x + y * y + z * z) * 39.3701;  // meters to inches
            }
        }

        // Fallback: ty angle estimation
        double ty = result.getTy();
        double angleToTargetRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + ty);
        double heightDiff = TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;

        if (Math.abs(angleToTargetRad) < 0.01) {
            return 0;
        }

        return Math.abs(heightDiff / Math.tan(angleToTargetRad));
    }

    private double interpolateShooterPower(double distance) {
        if (distance <= SHOOTER_DISTANCES[0]) {
            return SHOOTER_POWERS[0];
        }
        if (distance >= SHOOTER_DISTANCES[SHOOTER_DISTANCES.length - 1]) {
            return SHOOTER_POWERS[SHOOTER_POWERS.length - 1];
        }

        for (int i = 0; i < SHOOTER_DISTANCES.length - 1; i++) {
            if (distance >= SHOOTER_DISTANCES[i] && distance <= SHOOTER_DISTANCES[i + 1]) {
                double t = (distance - SHOOTER_DISTANCES[i]) /
                           (SHOOTER_DISTANCES[i + 1] - SHOOTER_DISTANCES[i]);
                return SHOOTER_POWERS[i] + t * (SHOOTER_POWERS[i + 1] - SHOOTER_POWERS[i]);
            }
        }

        return SHOOTER_POWER_DEFAULT;
    }

    // ======================== TELEMETRY ========================

    private void updateTelemetry() {
        // Drive telemetry
        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Mode", gamepad1.left_bumper ? "SLOW" : "Normal");

        // Intake telemetry
        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Power", "%.2f", frontIntake.getPower());

        // Shooter telemetry
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Power", "%.2f", shooter.getPower());
        telemetry.addData("Velocity", "%.0f ticks/s", shooter.getVelocity());
        telemetry.addData("Auto-Power", autoPowerEnabled ? "ON" : "OFF");
        telemetry.addData("Preset", "%d (%.0f%%)", presetIndex + 1, SHOOTER_PRESETS[presetIndex] * 100);
        telemetry.addData("Calculated Power", "%.2f", currentShooterPower);

        // Turret telemetry
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Position", "%.3f", turretPosition);
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ON" : "OFF");
        telemetry.addData("Locked", turretLocked ? "YES" : "No");

        // AprilTag telemetry
        telemetry.addLine("=== APRILTAG ===");
        if (hasAprilTag()) {
            telemetry.addData("ID", getFirstAprilTagId());
            telemetry.addData("TX", "%.2f deg", getAprilTagTx());
            telemetry.addData("TY", "%.2f deg", getAprilTagTy());
            telemetry.addData("Distance", "%.1f in", currentDistance);
            telemetry.addData("On Target", Math.abs(getAprilTagTx()) < AIM_TOLERANCE ? "YES" : "No");
        } else {
            telemetry.addData("Status", "NOT DETECTED");
        }

        // Pinpoint odometry telemetry
        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("Position", "X: %.1f  Y: %.1f in", robotX, robotY);
        telemetry.addData("Heading", "%.1f deg", robotHeading);
        telemetry.addData("Velocity", "%.1f in/s", Math.sqrt(robotVelocityX * robotVelocityX + robotVelocityY * robotVelocityY));
        telemetry.addData("AprilTag Correction", aprilTagCorrectionEnabled ? "ON" : "OFF");
        telemetry.addData("Corrections", correctionCount);
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus().toString());

        // Limelight status
        LLStatus status = limelight.getStatus();
        telemetry.addLine("=== LIMELIGHT ===");
        telemetry.addData("Pipeline", status.getPipelineIndex());
        telemetry.addData("FPS", "%.0f", status.getFps());

        // Pusher telemetry
        telemetry.addLine("=== PUSHER ===");
        telemetry.addData("Firing", isFiring ? "YES" : "No");

        // Controls reminder
        telemetry.addLine("=== GP1: DRIVER ===");
        telemetry.addData("Sticks", "Drive | LB=Slow");
        telemetry.addData("Triggers", "RT=Intake | LT=Reverse");
        telemetry.addData("X/B", "Toggle correction | Reset odom");

        telemetry.addLine("=== GP2: OPERATOR ===");
        telemetry.addData("RT", "Shooter (auto) | LT=Shooter (manual)");
        telemetry.addData("LB", "Fire | RB=Toggle aim");
        telemetry.addData("D-Pad", "U/D=Preset | L/R=Turret");
        telemetry.addData("A", "Toggle auto-power | Y=Reset | B=Stop");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        frontIntake.setPower(0);
        rearIntake.setPower(0);
        shooter.setPower(0);

        // Reset servos
        pusherServo.setPosition(PUSHER_RETRACTED);
        turretGear.setPosition(TURRET_HOME);

        // Stop Limelight
        limelight.stop();
    }
}
