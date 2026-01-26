package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Red Teleop", group = "Teleop")
public class RedTeleop extends OpMode {

    // ═══════════════════════════════════════════════════════════════════
    // DISTANCE-POWER LOOKUP TABLE
    // ═══════════════════════════════════════════════════════════════════
    // Distances in inches
    private static final double[] DISTANCE_PRESETS = {24.0, 36.0, 48.0, 60.0, 72.0, 120.0};
    // Corresponding power levels (0.0 - 1.0)
    private static final double[] POWER_PRESETS =    {0.55, 0.65, 0.60, 0.75, 0.85, 1.00};
    // Distance preset names for display
    private static final String[] DISTANCE_NAMES = {"2 ft", "3 ft", "4 ft", "5 ft", "6 ft", "10 ft"};

    private int currentDistanceIndex = 0;  // Start at 24"

    // ═══════════════════════════════════════════════════════════════════
    // AUTO-DISTANCE MODE
    // ═══════════════════════════════════════════════════════════════════
    private boolean autoDistanceMode = false;  // Toggle with Y button
    private double limelightDistance = 0;      // Distance from Limelight in inches
    private double autoPowerLevel = 0;         // Interpolated power level
    private boolean limelightHasTarget = false;

    // Target AprilTag ID for distance calculation (set to your scoring target)
    private static final int TARGET_APRILTAG_ID = 24;  // -1 = use any/closest tag

    // ═══════════════════════════════════════════════════════════════════

    // Pedro Pathing (for field-centric drive)
    private Follower follower;
    private Timer opmodeTimer;

    // Hardware - Drive motors handled by Pedro Pathing
    private DcMotor frontIntake;
    private DcMotor rearIntake;
    private DcMotorEx shooter;
    private Servo turretGear;
    private Servo pusherServo;

    // Limelight
    private Limelight3A limelight;
    private boolean limelightConnected = false;

    // ═══════════════════════════════════════════════════════════════════
    // SHOOTER CONFIGURATION (from BlueAutov3)
    // ═══════════════════════════════════════════════════════════════════

    // Motor specifications (goBILDA 5203 series 1:1 ratio)
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double TICKS_PER_SECOND_AT_MAX_RPM = (MOTOR_MAX_RPM / 60.0) * MOTOR_TICKS_PER_REV;

    // PIDF coefficients (from testing)
    private static final double SHOOTER_KP = 9.0;
    private static final double SHOOTER_KI = 0.100;
    private static final double SHOOTER_KD = 0.1;
    private static final double SHOOTER_KF = 12.5;

    // Velocity verification
    private static final double VELOCITY_TOLERANCE_PERCENT = 2.0;

    // ═══════════════════════════════════════════════════════════════════

    // Servo positions and motor powers
    private static final double INTAKE_POWER = 1.0;
    private static final double TURRET_CENTER = 0.5;
    private static final double TURRET_MIN = 0.2;
    private static final double TURRET_MAX = 0.8;
    private static final double PUSHER_RETRACTED = 0.2;
    private static final double PUSHER_EXTENDED = 0.5;

    // Limelight settings
    private static final int LIMELIGHT_PIPELINE = 6;

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
    private boolean lastRightBumper = false;
    private boolean lastYButton = false;

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
        rearIntake = hardwareMap.get(DcMotor.class, "rearIntake");
        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();
            limelightConnected = true;
        } catch (Exception e) {
            limelightConnected = false;
        }

        // Display initialization
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("RED TELEOP - AUTO DISTANCE");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("Distance-Power Table:");
        for (int i = 0; i < DISTANCE_PRESETS.length; i++) {
            telemetry.addData("  " + DISTANCE_NAMES[i],
                    String.format("%.0f%% power", POWER_PRESETS[i] * 100));
        }
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

        // Handle all controls
        handleDrive();
        handleAutoDistanceToggle();
        handleDistanceSelection();
        handleShooter();
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
     * Uses the 3D pose data to calculate distance in inches.
     */
    private void updateLimelightDistance() {
        limelightHasTarget = false;
        limelightDistance = 0;
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
            // Get distance from camera to tag using the robot-space pose
            // The pose gives us the tag's position relative to the robot
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
        double forward = -gamepad1.left_stick_y;  // Forward/backward
        double strafe = gamepad1.left_stick_x;    // Left/right
        double rotate = gamepad1.right_stick_x;   // Rotation

        // Apply deadzone
        if (Math.abs(forward) < 0.05) forward = 0;
        if (Math.abs(strafe) < 0.05) strafe = 0;
        if (Math.abs(rotate) < 0.05) rotate = 0;

        // Update Pedro Pathing teleop drive (forward, strafe, rotate, fieldCentric)
        follower.setTeleOpDrive(forward, strafe, rotate, true);
        follower.update();
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
    // SHOOTER CONTROL - GAMEPAD 2 (RIGHT TRIGGER)
    // ═══════════════════════════════════════════════════════════════════

    private void handleShooter() {
        // Right trigger: Spin up shooter at selected power level
        if (gamepad2.right_trigger > 0.5) {
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
                shooter.setVelocity(currentTargetVelocity);
            }
        }
    }

    private void stopShooter() {
        shooter.setPower(0);
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
    // PUSHER CONTROL - GAMEPAD 2 (RIGHT BUMPER)
    // ═══════════════════════════════════════════════════════════════════

    private void handlePusher() {
        boolean rightBumper = gamepad2.right_bumper;

        // Right bumper: Push sample
        if (rightBumper && !lastRightBumper) {
            pusherServo.setPosition(PUSHER_EXTENDED);
        } else if (!rightBumper && lastRightBumper) {
            pusherServo.setPosition(PUSHER_RETRACTED);
        }

        lastRightBumper = rightBumper;
    }

    // ═══════════════════════════════════════════════════════════════════
    // INTAKE CONTROL - GAMEPAD 2 (LEFT TRIGGER / LEFT BUMPER)
    // ═══════════════════════════════════════════════════════════════════

    private void handleIntake() {
        // Left trigger: Run intake forward
        if (gamepad2.left_trigger > 0.5) {
            frontIntake.setPower(INTAKE_POWER);
            rearIntake.setPower(INTAKE_POWER);
        }
        // Left bumper: Reverse intake
        else if (gamepad2.left_bumper) {
            frontIntake.setPower(-INTAKE_POWER);
            rearIntake.setPower(-INTAKE_POWER);
        }
        // No input: Stop intake
        else {
            frontIntake.setPower(0);
            rearIntake.setPower(0);
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // TURRET CONTROL - GAMEPAD 2 (D-PAD LEFT/RIGHT)
    // ═══════════════════════════════════════════════════════════════════

    private void handleTurret() {
        // D-pad Left/Right: Adjust turret position
        if (gamepad2.dpad_left) {
            turretPosition = Math.max(TURRET_MIN, turretPosition - TURRET_INCREMENT);
        } else if (gamepad2.dpad_right) {
            turretPosition = Math.min(TURRET_MAX, turretPosition + TURRET_INCREMENT);
        }

        // A button: Center turret
        if (gamepad2.a) {
            turretPosition = TURRET_CENTER;
        }

        turretGear.setPosition(turretPosition);
    }

    // ═══════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════

    private void displayTelemetry() {
        // Header
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("RED TELEOP");
        telemetry.addLine("═══════════════════════════════════════");

        // Auto-Distance Mode Status
        telemetry.addLine("\n─── DISTANCE MODE ───");
        telemetry.addData("Mode", autoDistanceMode ? "AUTO (Limelight)" : "MANUAL");
        telemetry.addLine("(Press Y to toggle)");

        if (autoDistanceMode) {
            // Show Limelight auto-distance info
            if (limelightHasTarget) {
                telemetry.addData("Detected Distance", String.format("%.1f\"", limelightDistance));
                telemetry.addData("Auto Power", String.format("%.0f%%", autoPowerLevel * 100));
            } else {
                telemetry.addData("Target", "NOT DETECTED");
                telemetry.addLine("  (Using last known or manual)");
            }
        } else {
            // Show manual preset selection
            telemetry.addData("Selected", DISTANCE_NAMES[currentDistanceIndex] +
                    " (" + (int)DISTANCE_PRESETS[currentDistanceIndex] + "\")");
            telemetry.addData("Power Level", String.format("%.0f%%", POWER_PRESETS[currentDistanceIndex] * 100));
        }

        // Distance-Power Table (compact)
        telemetry.addLine("\nDistance Table:");
        StringBuilder tableStr = new StringBuilder();
        for (int i = 0; i < DISTANCE_PRESETS.length; i++) {
            String marker = (!autoDistanceMode && i == currentDistanceIndex) ? ">" : " ";
            tableStr.append(String.format("%s%.0f\":%.0f%% ",
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
            telemetry.addData("Current RPM", String.format("%.0f", currentRPM));
            telemetry.addData("Target RPM", String.format("%.0f", targetRPM));
            telemetry.addData("Ready to Fire", shooterVelocityReached ? "YES" : "NO");
        }

        // Mechanisms
        telemetry.addLine("\n─── MECHANISMS ───");
        telemetry.addData("Intake", frontIntake.getPower() > 0 ? "FORWARD" :
                (frontIntake.getPower() < 0 ? "REVERSE" : "OFF"));
        telemetry.addData("Turret", String.format("%.2f", turretPosition));
        telemetry.addData("Pusher", pusherServo.getPosition() > 0.3 ? "EXTENDED" : "RETRACTED");

        // Limelight Info
        addLimelightTelemetry();

        // Controls Reference (compact)
        telemetry.addLine("\n─── CONTROLS ───");
        telemetry.addLine("GP1: Sticks=Drive");
        telemetry.addLine("GP2: Y=AutoMode, D-pad=Dist/Turret");
        telemetry.addLine("GP2: RT=Shoot, RB=Push, LT=Intake");

        telemetry.update();
    }

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

        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            telemetry.addData("Bot Pose", String.format("X:%.1f Y:%.1f",
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
        rearIntake.setPower(0);
        pusherServo.setPosition(PUSHER_RETRACTED);

        // Stop Limelight
        if (limelightConnected) {
            limelight.stop();
        }
    }
}