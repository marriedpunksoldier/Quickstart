package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autonomous Test OpMode
 *
 * Tests localization and path following using waypoints from BlueAutov3.
 * Use gamepad to select target waypoint, then robot will drive to it.
 *
 * Controls:
 *   D-pad Up/Down: Cycle through waypoints
 *   A button: Drive to selected waypoint
 *   B button: Stop current path
 *   X button: Reset pose to start position
 *   Y button: Drive through ALL waypoints sequentially
 */
@Autonomous(name = "Autonomous Test", group = "Test")
public class AutonomousTest extends OpMode {
    // Pedro Pathing
    private Follower follower;
    private Timer pathTimer;

    // Waypoint selection
    private int selectedWaypointIndex = 0;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;

    // Sequential path mode
    private boolean runningSequence = false;
    private int sequenceIndex = 0;

    // ==================== Poses from BlueAutov3 ====================
    // Start position
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    // All waypoints in order (for easy selection and sequential running)
    private final Pose[] waypoints = {
        new Pose(56, 8, Math.toRadians(0)),              // 0: Start
        new Pose(66.8995, 86.827, Math.toRadians(143)),   // 1: Shoot 1
        new Pose(53.14, 35.1104, Math.toRadians(180)),    // 2: Waypoint 1
        new Pose(29.1796, 35.5848, Math.toRadians(180)),  // 3: Waypoint 2
        new Pose(53.14, 35.1104, Math.toRadians(180)),    // 4: Waypoint 3
        new Pose(66.8995, 86.827, Math.toRadians(143)),   // 5: Shoot 2
        new Pose(52.6656, 60.0198, Math.toRadians(180)),  // 6: Waypoint 4
        new Pose(32.5008, 59.7825, Math.toRadians(180)),  // 7: Waypoint 5
        new Pose(52.6656, 60.0198, Math.toRadians(143)),  // 8: Waypoint 6
        new Pose(67.1367, 87.0643, Math.toRadians(143)),  // 9: Shoot 3
        new Pose(48.1582, 83.743, Math.toRadians(180)),   // 10: Waypoint 7
        new Pose(29.1796, 83.743, Math.toRadians(180)),   // 11: Waypoint 8
        new Pose(67.1367, 86.827, Math.toRadians(143))    // 12: Shoot 4
    };

    private final String[] waypointNames = {
        "Start",
        "Shoot 1",
        "Waypoint 1",
        "Waypoint 2",
        "Waypoint 3",
        "Shoot 2",
        "Waypoint 4",
        "Waypoint 5",
        "Waypoint 6",
        "Shoot 3",
        "Waypoint 7",
        "Waypoint 8",
        "Shoot 4"
    };

    // Path for current movement
    private PathChain currentPath = null;

    @Override
    public void init() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        // Initialize timer
        pathTimer = new Timer();

        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("AUTONOMOUS TEST");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("D-pad Up/Down: Select waypoint");
        telemetry.addLine("A: Go to selected waypoint");
        telemetry.addLine("B: Stop");
        telemetry.addLine("X: Reset to start pose");
        telemetry.addLine("Y: Run full sequence");
        telemetry.addLine("────────────────────────────────");
        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();

        // Handle input
        handleInput();

        // Handle sequential path mode
        if (runningSequence && !follower.isBusy()) {
            advanceSequence();
        }

        // Display telemetry
        displayTelemetry();
    }

    private void handleInput() {
        // D-pad Up - next waypoint
        if (gamepad1.dpad_up && !dpadUpPressed) {
            dpadUpPressed = true;
            selectedWaypointIndex = (selectedWaypointIndex + 1) % waypoints.length;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        // D-pad Down - previous waypoint
        if (gamepad1.dpad_down && !dpadDownPressed) {
            dpadDownPressed = true;
            selectedWaypointIndex = (selectedWaypointIndex - 1 + waypoints.length) % waypoints.length;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // A button - go to selected waypoint
        if (gamepad1.a && !aPressed) {
            aPressed = true;
            runningSequence = false;
            goToWaypoint(selectedWaypointIndex);
        } else if (!gamepad1.a) {
            aPressed = false;
        }

        // B button - stop
        if (gamepad1.b && !bPressed) {
            bPressed = true;
            runningSequence = false;
            follower.breakFollowing();
        } else if (!gamepad1.b) {
            bPressed = false;
        }

        // X button - reset pose to start
        if (gamepad1.x && !xPressed) {
            xPressed = true;
            runningSequence = false;
            follower.breakFollowing();
            follower.setPose(startPose);
            selectedWaypointIndex = 0;
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        // Y button - run full sequence
        if (gamepad1.y && !yPressed) {
            yPressed = true;
            startSequence();
        } else if (!gamepad1.y) {
            yPressed = false;
        }
    }

    private void goToWaypoint(int index) {
        if (index < 0 || index >= waypoints.length) return;

        Pose currentPose = follower.getPose();
        Pose targetPose = waypoints[index];

        // Build path from current position to target
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(currentPath, true);
        pathTimer.resetTimer();
    }

    private void startSequence() {
        runningSequence = true;
        sequenceIndex = 0;
        goToWaypoint(sequenceIndex);
    }

    private void advanceSequence() {
        sequenceIndex++;
        if (sequenceIndex < waypoints.length) {
            goToWaypoint(sequenceIndex);
        } else {
            // Sequence complete
            runningSequence = false;
            sequenceIndex = 0;
        }
    }

    private void displayTelemetry() {
        Pose currentPose = follower.getPose();
        Pose targetPose = waypoints[selectedWaypointIndex];

        // Header
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("AUTONOMOUS TEST");
        telemetry.addLine("════════════════════════════════");

        // Current position (from Pinpoint odometry)
        telemetry.addLine("─── CURRENT POSITION ───");
        telemetry.addData("X", "%.2f in", currentPose.getX());
        telemetry.addData("Y", "%.2f in", currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

        // Selected waypoint
        telemetry.addLine("─── SELECTED TARGET ───");
        telemetry.addData("Waypoint", "[%d] %s", selectedWaypointIndex, waypointNames[selectedWaypointIndex]);
        telemetry.addData("Target X", "%.2f in", targetPose.getX());
        telemetry.addData("Target Y", "%.2f in", targetPose.getY());
        telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetPose.getHeading()));

        // Distance to target
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        double headingError = Math.toDegrees(targetPose.getHeading() - currentPose.getHeading());
        // Normalize heading error to -180 to 180
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;

        telemetry.addLine("─── ERROR ───");
        telemetry.addData("Distance", "%.2f in", distance);
        telemetry.addData("Heading Error", "%.1f°", headingError);

        // Status
        telemetry.addLine("─── STATUS ───");
        if (runningSequence) {
            telemetry.addData("Mode", "SEQUENCE [%d/%d]", sequenceIndex + 1, waypoints.length);
        } else if (follower.isBusy()) {
            telemetry.addData("Mode", "MOVING");
        } else {
            telemetry.addData("Mode", "IDLE");
        }
        telemetry.addData("Path Time", "%.1f s", pathTimer.getElapsedTimeSeconds());

        // Controls reminder
        telemetry.addLine("────────────────────────────────");
        telemetry.addLine("D-pad: Select | A: Go | B: Stop");
        telemetry.addLine("X: Reset | Y: Run All");

        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}
