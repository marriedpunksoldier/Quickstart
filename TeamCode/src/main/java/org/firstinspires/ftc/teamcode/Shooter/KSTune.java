package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * KSTune — Step 1 of the shooter tuning pipeline.
 *
 * GOAL
 * ----
 * Find the smallest motor power that JUST overcomes static friction and
 * starts the flywheel rotating. That value is KS — the "voltage floor"
 * the feedforward controller adds to every output so its math doesn't
 * waste output range fighting friction.
 *
 * TWO-MOTOR NOTE
 * --------------
 * The flywheel is driven by two motors (shooter1 + shooter2). Both run
 * during tuning via the FlywheelMotors helper. KS found with only one
 * motor would be wrong for the real system.
 *
 * PROCEDURE
 * ---------
 *   1. Press PLAY. Motors start at 0 power.
 *   2. Press D-pad UP to ramp power up by KS_STEP each press.
 *   3. Watch the flywheel. The instant it JUST begins to rotate, stop.
 *   4. Press D-pad DOWN once to back off one step.
 *   5. The displayed "Applied Power" is your KS.
 *   6. Copy that value into ShooterConfig.KS_INITIAL and run KVTune.
 *
 * Controls (gamepad2):
 *   dpad_up    — increase applied power by KS_STEP
 *   dpad_down  — decrease applied power by KS_STEP
 *   left_bumper, back — emergency stop (power -> 0)
 */
@TeleOp(name = "Shooter - 1. KS Tune (Static Friction)", group = "Shooter Tuning")
public class KSTune extends LinearOpMode {

    private FlywheelMotors flywheel;
    private double appliedPower = 0.0;

    private final ElapsedTime dpadUpTimer    = new ElapsedTime();
    private final ElapsedTime dpadDownTimer  = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addLine("Initialising KS Tune...");
        telemetry.update();

        flywheel = new FlywheelMotors(hardwareMap);

        telemetry.addLine("Flywheel ready (2 motors). Press PLAY.");
        telemetry.addLine("Goal: find min power that JUST rotates the flywheel.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Emergency stop — checked first so it always wins
            if (gamepad2.left_bumper || gamepad2.back) {
                appliedPower = 0.0;
            }

            // Adjust applied power
            if (debounced(gamepad2.dpad_up, dpadUpTimer)) {
                appliedPower = Math.min(1.0, appliedPower + ShooterConfig.KS_STEP);
            }
            if (debounced(gamepad2.dpad_down, dpadDownTimer)) {
                appliedPower = Math.max(0.0, appliedPower - ShooterConfig.KS_STEP);
            }

            // Apply to both motors
            flywheel.setPower(appliedPower);

            // Read velocity from each encoder (signed — detect reversal)
            double signedRPM1 = flywheel.getRPM();
            double signedRPM2 = flywheel.getMotor2RPM();

            if (telemetryTimer.milliseconds() > ShooterConfig.TELEMETRY_UPDATE_MS) {
                updateTelemetry(signedRPM1, signedRPM2);
                telemetryTimer.reset();
            }
        }

        flywheel.setPower(0);
    }

    private boolean debounced(boolean pressed, ElapsedTime timer) {
        if (pressed && timer.milliseconds() > ShooterConfig.BUTTON_DEBOUNCE_MS) {
            timer.reset();
            return true;
        }
        return false;
    }

    private void updateTelemetry(double signedRPM1, double signedRPM2) {
        telemetry.addLine("=== KS TUNE (Step 1 of 3) ===");
        telemetry.addLine("Find min power that JUST rotates the flywheel.");
        telemetry.addLine();
        telemetry.addData("Applied Power (KS candidate)", "%.4f", appliedPower);
        telemetry.addData("Motor 1 RPM", "%.1f", signedRPM1);
        telemetry.addData("Motor 2 RPM", "%.1f", signedRPM2);
        telemetry.addLine();

        // Each motor should spin POSITIVE. Direction is configured in
        // FlywheelMotors (motor1 FORWARD, motor2 REVERSE). If either reads
        // negative under positive power, that motor's direction is wrong.
        if (signedRPM1 < -50) {
            telemetry.addLine(">>> MOTOR 1 REVERSED — check shooter1 direction <<<");
        }
        if (signedRPM2 < -50) {
            telemetry.addLine(">>> MOTOR 2 REVERSED — check shooter2 direction <<<");
        }
        // If the two motors spin OPPOSITE ways they fight each other —
        // catch that explicitly.
        if (signedRPM1 > 50 && signedRPM2 < -50) {
            telemetry.addLine(">>> MOTORS FIGHTING — shooter2 should be REVERSE <<<");
        }

        double absRPM = Math.abs(signedRPM1);
        if (absRPM > 5.0) {
            telemetry.addLine(">> Flywheel moving — press dpad_down once, copy power.");
        } else {
            telemetry.addLine("Not yet moving — press dpad_up to ramp.");
        }
        telemetry.addLine();
        telemetry.addLine("Controls: dpad_up/down to adjust, LB/back = STOP");
        telemetry.addLine();
        telemetry.addLine("Copy/paste:");
        telemetry.addData("=>", "public static final double KS_INITIAL = %.4f;", appliedPower);
        telemetry.update();
    }
}
