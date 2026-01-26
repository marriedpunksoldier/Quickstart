package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Standard OpMode for testing AprilTag tracking with Limelight 3A.
 * Displays AprilTag detection data including tag IDs, offsets, and robot pose.
 * Includes turret servo that tracks AprilTags in real-time.
 */
@TeleOp(name = "AprilTag Test", group = "Test")
public class AprilTagTest extends OpMode {

    private Limelight3A limelight;
    private Servo turretGear;

    // Pipeline for AprilTag detection (configure in Limelight web interface)
    private static final int APRILTAG_PIPELINE = 6;

    // Turret servo constants
    private static final double TURRET_CENTER = 0.5;      // Servo position when centered
    private static final double TURRET_MIN = 0.0;         // Minimum servo position
    private static final double TURRET_MAX = 1.0;         // Maximum servo position
    private static final double TURRET_RANGE_DEG = 90.0;  // Turret range in degrees
    private static final double AIM_TOLERANCE = 2.0;      // Degrees of acceptable error

    // Turret tracking state
    private boolean turretLocked = false;
    private double currentTurretPosition = TURRET_CENTER;

    @Override
    public void init() {
        // Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Initialize turret servo
        turretGear = hardwareMap.get(Servo.class, "turretGear");
        turretGear.setPosition(TURRET_CENTER);
        currentTurretPosition = TURRET_CENTER;

        // Set telemetry update rate
        telemetry.setMsTransmissionInterval(11);

        // Switch to AprilTag pipeline
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        // Start polling for data
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pipeline", APRILTAG_PIPELINE);
        telemetry.addData("Turret", "Centered at %.2f", TURRET_CENTER);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Show Limelight status during init
        LLStatus status = limelight.getStatus();
        telemetry.addData("Limelight", status.getName());
        telemetry.addData("Temp", "%.1fC", status.getTemp());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get Limelight status
        LLStatus status = limelight.getStatus();
        telemetry.addData("=== LIMELIGHT STATUS ===", "");
        telemetry.addData("Name", status.getName());
        telemetry.addData("Temp/CPU/FPS", "%.1fC / %.1f%% / %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        // Get latest result
        LLResult result = limelight.getLatestResult();

        // Track AprilTag with turret
        aimTurretAtAprilTag(result);

        // Turret status
        telemetry.addData("=== TURRET STATUS ===", "");
        telemetry.addData("Turret Position", "%.3f", currentTurretPosition);
        telemetry.addData("Turret Locked", turretLocked ? "YES" : "NO");

        if (result != null && result.isValid()) {
            // General targeting data
            telemetry.addData("=== TARGET DATA ===", "");
            telemetry.addData("TX (horizontal)", "%.2f deg", result.getTx());
            telemetry.addData("TY (vertical)", "%.2f deg", result.getTy());
            telemetry.addData("TX NC", "%.2f", result.getTxNC());
            telemetry.addData("TY NC", "%.2f", result.getTyNC());

            // Latency info
            double totalLatency = result.getCaptureLatency() + result.getTargetingLatency();
            telemetry.addData("Latency", "%.1f ms (capture + targeting)", totalLatency);

            // Robot pose from AprilTags
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                telemetry.addData("=== ROBOT POSE ===", "");
                telemetry.addData("Position", "X: %.2f, Y: %.2f, Z: %.2f",
                        botpose.getPosition().x,
                        botpose.getPosition().y,
                        botpose.getPosition().z);
                telemetry.addData("Rotation", "Yaw: %.2f, Pitch: %.2f, Roll: %.2f",
                        botpose.getOrientation().getYaw(),
                        botpose.getOrientation().getPitch(),
                        botpose.getOrientation().getRoll());
            }

            // AprilTag/Fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            telemetry.addData("=== APRILTAGS DETECTED ===", fiducialResults.size());

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Tag " + fr.getFiducialId(),
                        "X: %.2f, Y: %.2f, Area: %.2f",
                        fr.getTargetXDegrees(),
                        fr.getTargetYDegrees(),
                        fr.getTargetArea());

                // Show tag pose if available
                Pose3D tagPose = fr.getCameraPoseTargetSpace();
                if (tagPose != null) {
                    telemetry.addData("  Pose", "X: %.2f, Y: %.2f, Z: %.2f",
                            tagPose.getPosition().x,
                            tagPose.getPosition().y,
                            tagPose.getPosition().z);
                }
            }
        } else {
            telemetry.addData("=== NO APRILTAGS ===", "");
            telemetry.addData("Status", "No valid data - check pipeline config");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop Limelight polling
        if (limelight != null) {
            limelight.stop();
        }

        // Reset turret to center
        if (turretGear != null) {
            turretGear.setPosition(TURRET_CENTER);
        }
    }

    /**
     * Aim the turret at the detected AprilTag using Limelight data.
     * Converts the TX (horizontal offset) to a servo position.
     *
     * @param result The Limelight result containing target data
     */
    private void aimTurretAtAprilTag(LLResult result) {
        // Check for valid AprilTag detection
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            // No target - hold current position
            turretLocked = false;
            return;
        }

        double tx = result.getTx();

        // Check if we're within tolerance
        if (Math.abs(tx) < AIM_TOLERANCE) {
            turretLocked = true;
            return;
        }

        // Convert TX (degrees) to servo position adjustment
        // TX is positive when target is right, negative when left
        // Negate to move servo toward target (servo direction is inverted)
        double servoAdjustment = -(tx / TURRET_RANGE_DEG);
        double targetPosition = TURRET_CENTER + servoAdjustment;

        // Clamp to valid servo range
        targetPosition = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetPosition));

        // Update servo position
        turretGear.setPosition(targetPosition);
        currentTurretPosition = targetPosition;
        turretLocked = false;
    }
}
