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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * Hardware configuration:
 * - 4 drive motors (leftFront, rightFront, leftRear, rightRear) for mecanum drive
 * - 2 intake motors (frontIntake, rearIntake)
 * - 2 servos (pusherServo, turretGear)
 * - 1 GoBILDA Pinpoint Odometry Computer with two 4-Bar odometry pods
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, select this sample, and select TeleOp.
 *  Also add another new file named RobotHardware.java, select the sample with that name, and select Not an OpMode.
 */

@TeleOp(name="Concept: Robot Hardware Class", group="Robot")
@Disabled
public class ConceptExternalHardwareClass extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware   robot       = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive        = 0;
        double strafe       = 0;
        double turn         = 0;
        double intakePower  = 0;
        double pusherPos    = 0.7;
        double turretPos    = 0.0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Update odometry - must be called every loop iteration
            robot.updateOdometry();

            // Reset odometry position when B button is pressed
            if (gamepad1.b) {
                robot.resetPosition();
            }

            // Run wheels in mecanum mode (note: The joystick goes negative when pushed forward, so negate it)
            // Left stick controls forward/back and strafe, Right stick controls turning.
            drive  = -gamepad1.left_stick_y;
            strafe =  gamepad1.left_stick_x;
            turn   =  gamepad1.right_stick_x;

            // Drive the robot using mecanum. Use RobotHardware class
            robot.driveRobot(drive, strafe, turn);

            // Use triggers to control intake motors
            // Right trigger = intake, Left trigger = outtake
            if (gamepad1.right_trigger > 0.1)
                intakePower = gamepad1.right_trigger;
            else if (gamepad1.left_trigger > 0.1)
                intakePower = -gamepad1.left_trigger;
            else
                intakePower = 0;

            robot.setIntakePower(intakePower);

            // Use bumpers to control pusher servo
            if (gamepad1.right_bumper)
                pusherPos = 1.0;
            else if (gamepad1.left_bumper)
                pusherPos = 0.0;

            robot.setPusherPosition(pusherPos);

            // Use Y and A buttons to control turret servo
            if (gamepad1.y)
                turretPos = 1.0;
            else if (gamepad1.a)
                turretPos = 0.0;

            robot.setTurretPosition(turretPos);

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick Y");
            telemetry.addData("Strafe", "Left Stick X");
            telemetry.addData("Turn", "Right Stick X");
            telemetry.addData("Intake", "Triggers (R=in, L=out)");
            telemetry.addData("Pusher", "Bumpers (R=extend, L=retract)");
            telemetry.addData("Turret", "Y & A Buttons");
            telemetry.addData("Reset Odometry", "B Button");
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Strafe Power", "%.2f", strafe);
            telemetry.addData("Turn Power",  "%.2f", turn);
            telemetry.addData("Intake Power",  "%.2f", intakePower);
            telemetry.addData("Pusher Position",  "%.2f", pusherPos);
            telemetry.addData("Turret Position",  "%.2f", turretPos);
            telemetry.addData("-", "-------");

            // Odometry telemetry
            telemetry.addData("X (in)", "%.2f", robot.getX());
            telemetry.addData("Y (in)", "%.2f", robot.getY());
            telemetry.addData("Heading (deg)", "%.2f", robot.getHeading());
            telemetry.update();

            // Pace this loop so servos move at a reasonable speed.
            sleep(50);
        }
    }
}
