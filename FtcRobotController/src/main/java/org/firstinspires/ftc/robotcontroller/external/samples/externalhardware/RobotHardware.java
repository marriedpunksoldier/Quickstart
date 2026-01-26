/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
 * This file defines a Java Class that performs all the setup and configuration for the robot's hardware.
 *
 * Hardware configuration:
 * - 4 drive motors (leftFront, rightFront, leftRear, rightRear) for mecanum drive
 * - 2 intake motors (frontIntake, rearIntake)
 * - 1 shooter motor (shooter)
 * - 2 servos (pusherServo, turretGear)
 * - 1 GoBILDA Pinpoint Odometry Computer with two 4-Bar odometry pods
 * - 1 Limelight 3A vision sensor for AprilTag detection
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor frontIntake   = null;
    private DcMotor rearIntake   = null;
    private DcMotor shooter =  null;
    private Servo   pusherServo = null;
    private Servo   turretGear = null;

    // GoBILDA Pinpoint Odometry Computer
    private GoBildaPinpointDriver pinpoint = null;

    // Limelight 3A Vision Sensor
    private Limelight3A limelight = null;

    // Odometry pod offset constants (in mm)
    // X pod offset: how far sideways from center the forward (X) pod is. Left is positive.
    // Y pod offset: how far forwards from center the strafe (Y) pod is. Forward is positive.
    public static final double X_POD_OFFSET_MM = -25.4;   // Adjust based on your robot
    public static final double Y_POD_OFFSET_MM = -203.2;  // Adjust based on your robot


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFront  = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = myOpMode.hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = myOpMode.hardwareMap.get(DcMotor.class, "rightRear");
        frontIntake  = myOpMode.hardwareMap.get(DcMotor.class, "frontIntake");
        rearIntake  = myOpMode.hardwareMap.get(DcMotor.class, "rearIntake");
        shooter     = myOpMode.hardwareMap.get(DcMotor.class, "shooter");
        // For mecanum drive, left motors are typically reversed since axles point in opposite directions.
        // Adjust these if your robot drives incorrectly during testing.
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Set all drive motors to brake when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontIntake.setDirection(DcMotor.Direction.FORWARD);
        rearIntake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        // Define and initialize ALL installed servos.
        pusherServo = myOpMode.hardwareMap.get(Servo.class, "pusherServo");
        pusherServo.setPosition(0.7);
        turretGear = myOpMode.hardwareMap.get(Servo.class, "turretGear");
        turretGear.setPosition(0.5);

        // Initialize GoBILDA Pinpoint Odometry Computer
        pinpoint = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure odometry pod offsets (distance from robot center to each pod)
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);

        // Set encoder resolution for GoBILDA 4-Bar Odometry Pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions (adjust if robot tracks incorrectly)
        // X (forward) pod should increase when moving forward
        // Y (strafe) pod should increase when moving left
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Recalibrate IMU and reset position to origin
        pinpoint.resetPosAndIMU();

        // Initialize Limelight 3A Vision Sensor
        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // Use pipeline 0 (configure in Limelight web interface)
        limelight.start();  // Start polling for data

        // Send telemetry message to signify robot is ready.
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the motor powers required to achieve the requested robot motions
     * for a mecanum drive: Drive (Axial), Strafe (Lateral), and Turn (Yaw).
     * Then sends these power levels to all four motors.
     *
     * @param drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Left/Right strafing power (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double drive, double strafe, double turn) {
        // Calculate power for each mecanum wheel
        double leftFrontPower  = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftRearPower   = drive - strafe + turn;
        double rightRearPower  = drive + strafe - turn;

        // Normalize the values so no wheel power exceeds 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftRearPower   /= max;
            rightRearPower  /= max;
        }

        // Send power to motors
        setDrivePower(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }

    /**
     * Pass the requested wheel motor powers to all four mecanum drive motors.
     *
     * @param leftFrontPower   Power for left front motor (-1.0 to 1.0)
     * @param rightFrontPower  Power for right front motor (-1.0 to 1.0)
     * @param leftRearPower    Power for left rear motor (-1.0 to 1.0)
     * @param rightRearPower   Power for right rear motor (-1.0 to 1.0)
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower,
                              double leftRearPower, double rightRearPower) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    /**
     * Stop all drive motors.
     */
    public void stopDriving() {
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * Set the intake motors power.
     *
     * @param power intake power (-1.0 to 1.0) +ve is intake, -ve is outtake
     */
    public void setIntakePower(double power) {
        frontIntake.setPower(power);
        rearIntake.setPower(power);
    }

    /**
     * Set the shooter motor power.
     *
     * @param power shooter power (-1.0 to 1.0)
     */
    public void setShooter(double power) {
        shooter.setPower(power);
    }

    /**
     * Set the pusher servo position.
     *
     * @param position servo position (clamped to 0.7 to 1.0)
     */
    public void setPusherPosition(double position) {
        pusherServo.setPosition(Range.clip(position, 0.7, 1.0));
    }

    /**
     * Set the turret gear servo position.
     *
     * @param position servo position (0.0 to 1.0)
     */
    public void setTurretPosition(double position) {
        turretGear.setPosition(Range.clip(position, 0.0, 1.0));
    }

    /**
     * Update the Pinpoint odometry. Must be called once per loop iteration.
     */
    public void updateOdometry() {
        pinpoint.update();
    }

    /**
     * Get the current robot position from the Pinpoint odometry.
     *
     * @return Pose2D containing x, y position and heading
     */
    public Pose2D getPosition() {
        return pinpoint.getPosition();
    }

    /**
     * Get the X coordinate from Pinpoint odometry.
     *
     * @return X position in inches
     */
    public double getX() {
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }

    /**
     * Get the Y coordinate from Pinpoint odometry.
     *
     * @return Y position in inches
     */
    public double getY() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    /**
     * Get the heading from Pinpoint odometry.
     *
     * @return heading in degrees
     */
    public double getHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    /**
     * Reset the Pinpoint position to a new pose.
     *
     * @param x       X position in inches
     * @param y       Y position in inches
     * @param heading heading in degrees
     */
    public void setPosition(double x, double y, double heading) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
    }

    /**
     * Reset the Pinpoint position to origin (0, 0, 0) and recalibrate IMU.
     */
    public void resetPosition() {
        pinpoint.resetPosAndIMU();
    }

    // ======================== LIMELIGHT METHODS ========================

    /**
     * Get the latest result from the Limelight.
     *
     * @return LLResult containing vision data, or null if no data available
     */
    public LLResult getLimelightResult() {
        return limelight.getLatestResult();
    }

    /**
     * Check if the Limelight has a valid target.
     *
     * @return true if a valid result is available
     */
    public boolean hasLimelightTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    /**
     * Get the horizontal offset to target (tx) in degrees.
     *
     * @return tx value, or 0 if no valid target
     */
    public double getLimelightTx() {
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
    public double getLimelightTy() {
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
    public Pose3D getLimelightBotpose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    /**
     * Get detected AprilTag/fiducial results.
     *
     * @return List of fiducial results, or empty list if none detected
     */
    public List<LLResultTypes.FiducialResult> getAprilTagResults() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return java.util.Collections.emptyList();
    }

    /**
     * Switch the Limelight pipeline.
     *
     * @param pipelineIndex pipeline number (0-9)
     */
    public void setLimelightPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Stop the Limelight. Call this when OpMode stops.
     */
    public void stopLimelight() {
        limelight.stop();
    }
}
