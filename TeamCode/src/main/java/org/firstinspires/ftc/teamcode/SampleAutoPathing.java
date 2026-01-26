package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Sample Auto Pathing", group="Samples")
@Disabled
public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer;

    // Hardware declarations
    private DcMotor intakeMotor;
    private DcMotor shooterMotor;
    private Servo shooterServo;

    // Hardware constants - adjust these values for your robot
    private static final double INTAKE_POWER = 1.0;
    private static final double SHOOTER_POWER = 0.8;
    private static final double SERVO_LOAD_POSITION = 0.0;    // Position to load specimen
    private static final double SERVO_SHOOT_POSITION = 1.0;   // Position to shoot specimen
    private static final double SHOOT_DELAY = 2.0;            // Time to complete shooting action

    public enum PathState {
        DRIVE_TO_SHOOT1,    // Drive from start to first shoot position
        SHOOT_SPECIMEN1,    // Shoot first specimen
        DRIVE_TO_SHOOT2,    // Drive to second shoot position (through waypoints)
        SHOOT_SPECIMEN2,    // Shoot second specimen
        DRIVE_TO_SHOOT3,    // Drive to third shoot position (through waypoints)
        SHOOT_SPECIMEN3,    // Shoot third specimen
        DRIVE_TO_END        // Drive to end position (through waypoints)
    }

    PathState pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(58.35914332784185, 96.3163097199341, Math.toRadians(143));

    private final Pose waypoint1 = new Pose(40.566721581548606, 35.58484349258651, Math.toRadians(180));

    private final Pose waypoint2 = new Pose(17.555189456342667, 35.3476112026359, Math.toRadians(180));

    private final Pose waypoint3 = new Pose(40.803953871499175, 35.58484349258651, Math.toRadians(180));

    private final Pose shootPose2 = new Pose(58.121911037891266, 96.79077429983526, Math.toRadians(143));

    private final Pose waypoint4 = new Pose(43.176276771004936, 59.78253706754529, Math.toRadians(180));

    private final Pose waypoint5 = new Pose(16.369028006589787, 59.30807248764415, Math.toRadians(180));

    private final Pose waypoint6 = new Pose(43.176276771004936, 59.78253706754529, Math.toRadians(143));

    private final Pose shootPose3 = new Pose(58.121911037891266, 96.79077429983526, Math.toRadians(143));

    private final Pose waypoint7 = new Pose(43.650741350906095, 83.98023064250413, Math.toRadians(180));

    private final Pose waypoint8 = new Pose(16.13179571663921, 84.45469522240526, Math.toRadians(180));

    private final Pose  endPose = new Pose(58.1219110378912666, 96.79077429983526, Math.toRadians(143));


    private PathChain path1, path2, path3, path4;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, waypoint1))
                .addPath(new BezierLine(waypoint1, waypoint2))
                .addPath(new BezierLine(waypoint2, waypoint3))
                .addPath(new BezierLine(waypoint3, shootPose2))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), shootPose2.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, waypoint4))
                .addPath(new BezierLine(waypoint4, waypoint5))
                .addPath(new BezierLine(waypoint5, waypoint6))
                .addPath(new BezierLine(waypoint6, shootPose3))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), shootPose3.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, waypoint7))
                .addPath(new BezierLine(waypoint7, waypoint8))
                .addPath(new BezierLine(waypoint8, endPose))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_TO_SHOOT1:
                // Optionally run intake while driving to collect specimens
                // intakeOn();

                if (!follower.isBusy()) {
                    intakeOff();  // Stop intake before shooting
                    setPathState(PathState.SHOOT_SPECIMEN1);
                }
                break;

            case SHOOT_SPECIMEN1:
                // Execute shooting sequence
                executeShootSequence();

                // Wait for shooting to complete
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY) {
                    follower.followPath(path2, true);
                    setPathState(PathState.DRIVE_TO_SHOOT2);
                }
                break;

            case DRIVE_TO_SHOOT2:
                // Run intake to collect next specimen
                intakeOn();

                if (!follower.isBusy()) {
                    intakeOff();
                    setPathState(PathState.SHOOT_SPECIMEN2);
                }
                break;

            case SHOOT_SPECIMEN2:
                executeShootSequence();

                if (pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY) {
                    follower.followPath(path3, true);
                    setPathState(PathState.DRIVE_TO_SHOOT3);
                }
                break;

            case DRIVE_TO_SHOOT3:
                // Run intake to collect next specimen
                intakeOn();

                if (!follower.isBusy()) {
                    intakeOff();
                    setPathState(PathState.SHOOT_SPECIMEN3);
                }
                break;

            case SHOOT_SPECIMEN3:
                executeShootSequence();

                if (pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY) {
                    follower.followPath(path4, true);
                    setPathState(PathState.DRIVE_TO_END);
                }
                break;

            case DRIVE_TO_END:
                if (!follower.isBusy()) {
                    // Stop all mechanisms
                    intakeOff();
                    shooterStop();
                    telemetry.addLine("Autonomous Complete!");
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    // ========== Hardware Control Methods ==========

    /**
     * Activates the intake motor to collect specimens
     */
    private void intakeOn() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    /**
     * Stops the intake motor
     */
    private void intakeOff() {
        intakeMotor.setPower(0);
    }

    /**
     * Runs intake in reverse (for ejecting)
     */
    private void intakeReverse() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    /**
     * Spins up the shooter motor to shooting speed
     */
    private void shooterSpinUp() {
        shooterMotor.setPower(SHOOTER_POWER);
    }

    /**
     * Stops the shooter motor
     */
    private void shooterStop() {
        shooterMotor.setPower(0);
    }

    /**
     * Shoots a specimen by moving servo to shoot position
     */
    private void shootSpecimen() {
        shooterServo.setPosition(SERVO_SHOOT_POSITION);
    }

    /**
     * Reloads shooter by moving servo to load position
     */
    private void reloadShooter() {
        shooterServo.setPosition(SERVO_LOAD_POSITION);
    }

    /**
     * Complete shooting sequence: spin up, shoot, wait, reload
     */
    private void executeShootSequence() {
        double elapsedTime = pathTimer.getElapsedTimeSeconds();

        if (elapsedTime < 0.5) {
            // Spin up shooter motor
            shooterSpinUp();
        } else if (elapsedTime < 1.0) {
            // Fire servo
            shootSpecimen();
        } else if (elapsedTime < 1.5) {
            // Keep shooter running
            shooterSpinUp();
        } else {
            // Reload and stop
            reloadShooter();
            shooterStop();
        }
    }

    // ========== Autonomous State Machine ==========

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Initialize hardware - use names from your robot configuration XML
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        shooterServo = hardwareMap.servo.get("shooterServo");

        // Configure motor directions (adjust as needed for your robot)
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servo to load position
        shooterServo.setPosition(SERVO_LOAD_POSITION);

        buildPaths();
        follower.setPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start the first path and set initial state
        follower.followPath(path1, true);
        setPathState(PathState.DRIVE_TO_SHOOT1);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // Path telemetry
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Is Busy", follower.isBusy());

        // Mechanism telemetry
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.addData("Servo Position", shooterServo.getPosition());

        telemetry.update();
    }
}
