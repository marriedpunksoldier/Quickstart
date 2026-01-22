package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * FlywheelTest - Test OpMode for flywheel shooter with AprilTag tracking
 *
 * Controls:
 *   Right Trigger: Spin up flywheel (proportional power)
 *   Left Bumper: Fire (actuate pusher servo)
 *   Right Bumper: Toggle auto-aim mode
 *   D-Pad Left/Right: Manual turret control
 *   D-Pad Up/Down: Cycle Limelight pipeline
 *   Y: Reset turret to center
 *   A: Cycle through preset flywheel speeds
 *
 * Hardware:
 *   - shooter: DcMotorEx (flywheel motor)
 *   - pusher: Servo (ball pusher)
 *   - turret: Servo (turret rotation)
 *   - limelight: Limelight3A (AprilTag tracking)
 */
@TeleOp(name = "Flywheel Test", group = "Test")
public class FlywheelTest extends LinearOpMode {

    // Hardware
    private DcMotorEx shooter;
    private Servo pusherServo;
    private Servo turretGear;
    private Limelight3A limelight;

    // Shooter constants
    private static final double SHOOTER_MAX_POWER = 1.0;
    private static final double[] PRESET_POWERS = {0.7, 0.75, 0.8, 0.85, 1.0};
    private int presetIndex = 2; // Start at 0.7 power

    // Pusher constants
    private static final double PUSHER_RETRACTED = 0.2;
    private static final double PUSHER_EXTENDED = 0.5;

    // Turret aiming constants (matching BlueAuto.java)
    private static final double TURRET_CENTER = 0.5;      // Servo position when centered
    private static final double TURRET_MIN = 0.0;         // Minimum servo position
    private static final double TURRET_MAX = 1.0;         // Maximum servo position
    private static final double TURRET_RANGE_DEG = 90.0;  // Turret range in degrees
    private static final double AIM_TOLERANCE = 2.0;      // Degrees of acceptable error

    // Limelight pipeline (default)
    private static final int LIMELIGHT_PIPELINE = 5;

    private double turretPosition = TURRET_CENTER;
    private boolean turretLocked = false;

    // State
    private boolean autoAimEnabled = false;
    private boolean lastRightBumper = false;
    private boolean lastAButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean isFiring = false;
    private long fireStartTime = 0;
    private static final long FIRE_DURATION_MS = 300;

    // Pipeline control
    private static final int MAX_PIPELINE = 9;
    private int currentPipeline = LIMELIGHT_PIPELINE;

    // AprilTag tracking data
    private double targetTx = 0;
    private double targetTy = 0;
    private double targetDistance = 0;
    private int targetId = -1;
    private boolean targetValid = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update AprilTag tracking
            updateLimelight();

            // Handle controls
            handleShooterControls();
            handleTurretControls();
            handlePusherControls();
            handlePipelineControls();

            // Update telemetry
            updateTelemetry();

            // Small delay to prevent overwhelming the system
            sleep(10);
        }

        // Cleanup
        shooter.setPower(0);
        limelight.stop();
    }

    private void initHardware() {
        // Initialize shooter motor
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize pusher servo
        pusherServo = hardwareMap.get(Servo.class, "pusherServo");
        pusherServo.setPosition(PUSHER_RETRACTED);

        // Initialize turret servo
        turretGear = hardwareMap.get(Servo.class, "turretGear");
        turretGear.setPosition(TURRET_CENTER);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
    }

    private void updateLimelight() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Get fiducial (AprilTag) results
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (!fiducials.isEmpty()) {
                // Use result.getTx()/getTy() for turret aiming (matches BlueAuto.java)
                targetTx = result.getTx();
                targetTy = result.getTy();

                // Get AprilTag ID from fiducial results
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                targetId = tag.getFiducialId();

                // Calculate distance from camera pose if available
                Pose3D cameraPose = tag.getCameraPoseTargetSpace();
                if (cameraPose != null) {
                    double x = cameraPose.getPosition().x;
                    double y = cameraPose.getPosition().y;
                    double z = cameraPose.getPosition().z;
                    targetDistance = Math.sqrt(x * x + y * y + z * z);
                } else {
                    // Estimate distance from ty angle (rough approximation)
                    targetDistance = estimateDistanceFromTy(targetTy);
                }

                targetValid = true;
            } else {
                targetValid = false;
                turretLocked = false;
            }
        } else {
            targetValid = false;
            turretLocked = false;
        }
    }

    private double estimateDistanceFromTy(double ty) {
        // Simple distance estimation based on vertical angle
        // Adjust these constants based on your camera mounting
        double cameraHeightInches = 13.5;
        double targetHeightInches = 30.0;
        double cameraMountAngleDeg = 15.0;

        double angleToTargetRad = Math.toRadians(cameraMountAngleDeg + ty);
        double heightDiff = targetHeightInches - cameraHeightInches;

        if (Math.abs(angleToTargetRad) < 0.01) {
            return 0;
        }

        return heightDiff / Math.tan(angleToTargetRad);
    }

    private void handleShooterControls() {
        // Right trigger controls flywheel power (proportional)
        double triggerPower = gamepad1.right_trigger;

        // A button cycles through preset powers
        if (gamepad1.a && !lastAButton) {
            presetIndex = (presetIndex + 1) % PRESET_POWERS.length;
        }
        lastAButton = gamepad1.a;

        // Use trigger for manual control, or preset if trigger not pressed
        double shooterPower;
        if (triggerPower > 0.1) {
            shooterPower = triggerPower * SHOOTER_MAX_POWER;
        } else if (gamepad1.left_trigger > 0.5) {
            // Left trigger uses current preset
            shooterPower = PRESET_POWERS[presetIndex];
        } else {
            shooterPower = 0;
        }

        shooter.setPower(shooterPower);
    }

    private void handleTurretControls() {
        // Toggle auto-aim with right bumper
        if (gamepad1.right_bumper && !lastRightBumper) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastRightBumper = gamepad1.right_bumper;

        // Y button resets turret to center
        if (gamepad1.y) {
            turretPosition = TURRET_CENTER;
            autoAimEnabled = false;
            turretLocked = false;
        }

        if (autoAimEnabled && targetValid) {
            // Auto-aim using BlueAuto.java's aimTurretAtAprilTag() logic
            // Check if we're within tolerance
            if (Math.abs(targetTx) < AIM_TOLERANCE) {
                turretLocked = true;
            } else {
                // Convert tx (degrees) to servo position adjustment
                // tx is positive when target is right, negative when left
                // Negate to move servo toward target (servo direction is inverted)
                double servoAdjustment = -(targetTx / TURRET_RANGE_DEG);
                turretPosition = TURRET_CENTER + servoAdjustment;
                turretLocked = false;
            }
        } else {
            // Manual turret control with D-pad
            if (gamepad1.dpad_left) {
                turretPosition -= 0.01;
            } else if (gamepad1.dpad_right) {
                turretPosition += 0.01;
            }
            turretLocked = false;
        }

        // Clamp turret position
        turretPosition = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretPosition));

        turretGear.setPosition(turretPosition);
    }

    private void handlePusherControls() {
        // Left bumper fires (extends pusher momentarily)
        if (gamepad1.left_bumper && !isFiring) {
            isFiring = true;
            fireStartTime = System.currentTimeMillis();
            pusherServo.setPosition(PUSHER_EXTENDED);
        }

        // Auto-retract pusher after fire duration
        if (isFiring && (System.currentTimeMillis() - fireStartTime) > FIRE_DURATION_MS) {
            pusherServo.setPosition(PUSHER_RETRACTED);
            isFiring = false;
        }
    }

    private void handlePipelineControls() {
        // D-Pad Up increases pipeline
        if (gamepad1.dpad_up && !lastDpadUp) {
            currentPipeline = (currentPipeline + 1) % (MAX_PIPELINE + 1);
            limelight.pipelineSwitch(currentPipeline);
        }
        lastDpadUp = gamepad1.dpad_up;

        // D-Pad Down decreases pipeline
        if (gamepad1.dpad_down && !lastDpadDown) {
            currentPipeline = (currentPipeline - 1 + MAX_PIPELINE + 1) % (MAX_PIPELINE + 1);
            limelight.pipelineSwitch(currentPipeline);
        }
        lastDpadDown = gamepad1.dpad_down;
    }

    private void updateTelemetry() {
        // Shooter telemetry
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Power", "%.2f", shooter.getPower());
        telemetry.addData("Velocity (ticks/s)", "%.0f", shooter.getVelocity());
        telemetry.addData("Preset", "%d (%.0f%%)", presetIndex + 1, PRESET_POWERS[presetIndex] * 100);

        // Pusher telemetry
        telemetry.addLine("=== PUSHER ===");
        telemetry.addData("Position", "%.2f", pusherServo.getPosition());
        telemetry.addData("Firing", isFiring ? "YES" : "No");

        // Turret telemetry
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Position", "%.3f", turretPosition);
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "Disabled");
        telemetry.addData("Turret Locked", turretLocked ? "YES" : "No");

        // Limelight / AprilTag telemetry
        telemetry.addLine("=== APRILTAG ===");
        if (targetValid) {
            telemetry.addData("Target ID", targetId);
            telemetry.addData("TX (horizontal)", "%.2f deg", targetTx);
            telemetry.addData("TY (vertical)", "%.2f deg", targetTy);
            telemetry.addData("Distance", "%.1f in", targetDistance);
            telemetry.addData("On Target", Math.abs(targetTx) < AIM_TOLERANCE ? "YES" : "No");
        } else {
            telemetry.addData("Target", "NOT DETECTED");
        }

        // Limelight status
        LLStatus status = limelight.getStatus();
        telemetry.addLine("=== LIMELIGHT ===");
        telemetry.addData("Pipeline (set)", currentPipeline);
        telemetry.addData("Pipeline (actual)", status.getPipelineIndex());
        telemetry.addData("Pipeline Type", status.getPipelineType());
        telemetry.addData("FPS", "%.0f", status.getFps());
        telemetry.addData("Temp", "%.1f C", status.getTemp());

        // Controls reminder
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("RT", "Flywheel (proportional)");
        telemetry.addData("LT", "Flywheel (preset)");
        telemetry.addData("LB", "Fire");
        telemetry.addData("RB", "Toggle auto-aim");
        telemetry.addData("D-Pad L/R", "Manual turret");
        telemetry.addData("D-Pad U/D", "Cycle pipeline");
        telemetry.addData("A", "Cycle presets");
        telemetry.addData("Y", "Reset turret");

        telemetry.update();
    }
}
