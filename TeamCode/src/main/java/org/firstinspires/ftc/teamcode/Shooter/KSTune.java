package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * KSTune.java — Step 1 of 3 in the shooter tuning pipeline.
 *
 * PURPOSE
 * -------
 * Find KS: the minimum motor power fraction required to overcome static
 * friction and get the flywheel physically moving from a standstill.
 *
 * WHY THIS MATTERS (from the video)
 * ----------------------------------
 * "Our controller assumes zero volts = zero RPM, but in reality you might
 *  have to send 0.5 V just to get the flywheel off the ground."
 *
 * Without KS, the feedforward controller's power scale starts from zero,
 * which means at low commanded RPMs the motor gets insufficient power to
 * even turn the flywheel — wasting the first fraction of its output range
 * on friction that delivers no motion.
 *
 * KS shifts the entire power curve upward by a constant, so that any
 * commanded velocity immediately results in actual movement.
 *
 * TUNING PROCEDURE
 * ----------------
 *   1. Press PLAY. The flywheel starts with zero power applied.
 *   2. Watch the flywheel wheel physically (or watch "Actual RPM" on telemetry).
 *   3. Press dpad_up repeatedly to increase applied power in small steps (KS_STEP).
 *   4. Stop the INSTANT the flywheel JUST begins to rotate (first tick of movement).
 *   5. Press dpad_down ONCE to back off one step.
 *   6. That value shown as "Current Power" on telemetry is your KS.
 *   7. Copy it into ShooterConfig.KS_INITIAL, then run KVTune.java.
 *
 * NOTE: Do NOT press right_bumper during this OpMode.  You are NOT spinning
 * up to target RPM here — you are only finding the friction floor.
 *
 * CONTROLS (gamepad2 only)
 * -------------------------
 *   dpad_up    → increase applied power by KS_STEP
 *   dpad_down  → decrease applied power by KS_STEP (min 0.0)
 *   left_bumper → stop motor (reset to 0 power)
 */
@TeleOp(name = "Shooter — 1. KS Tune  (Static Friction)", group = "Shooter Tuning")
public class KSTune extends LinearOpMode {

    private DcMotorEx flywheelMotor;

    // Current test power level — this IS the KS candidate
    private double appliedPower = 0.0;

    private final ElapsedTime dpadUpTimer   = new ElapsedTime();
    private final ElapsedTime dpadDownTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final long DEBOUNCE_MS = 250;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing KS Tune...");
        telemetry.update();

        flywheelMotor = initFlywheelMotor();

        telemetry.addLine("✓ Flywheel motor ready.");
        telemetry.addLine("Press PLAY to begin KS tuning.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Increase power
            if (gamepad2.dpad_up && dpadUpTimer.milliseconds() > DEBOUNCE_MS) {
                appliedPower = Math.min(1.0, appliedPower + ShooterConfig.KS_STEP);
                dpadUpTimer.reset();
            }

            // Decrease power
            if (gamepad2.dpad_down && dpadDownTimer.milliseconds() > DEBOUNCE_MS) {
                appliedPower = Math.max(0.0, appliedPower - ShooterConfig.KS_STEP);
                dpadDownTimer.reset();
            }

            // Emergency stop
            if (gamepad2.left_bumper) {
                appliedPower = 0.0;
            }

            // Apply power directly — bypass velocity controller, use raw power mode
            // so we see the pure friction floor without any PID assistance.
            flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelMotor.setPower(appliedPower);

            // Read actual velocity just for display (even in open-loop mode)
            double actualVelocityTPS = flywheelMotor.getVelocity();
            double actualRPM         = ticksPerSecondToRPM(Math.abs(actualVelocityTPS));

            if (telemetryTimer.milliseconds() > ShooterConfig.TELEMETRY_UPDATE_MS) {
                updateTelemetry(actualRPM);
                telemetryTimer.reset();
            }
        }

        shutdown();
    }

    private void updateTelemetry(double actualRPM) {
        telemetry.addLine("=== KS TUNE — Static Friction Floor ===");
        telemetry.addLine("(Step 1 of 3)");
        telemetry.addLine();
        telemetry.addLine("GOAL: Find the minimum power that makes the flywheel");
        telemetry.addLine("      JUST start to rotate, then back off one step.");
        telemetry.addLine();
        telemetry.addData("Applied Power (KS candidate)", "%.4f", appliedPower);
        telemetry.addData("Actual RPM",                   "%.1f", actualRPM);
        telemetry.addLine();
        telemetry.addData("Motor moving?", actualRPM > 5.0 ? "YES ← stop here, back off 1 step"
                                                           : "no — keep pressing dpad_up");
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("dpad_up    → +KS_STEP (" + ShooterConfig.KS_STEP + ")");
        telemetry.addLine("dpad_down  → -KS_STEP");
        telemetry.addLine("left_bumper → stop motor");
        telemetry.addLine();
        telemetry.addLine("When motor JUST moves: press dpad_down once,");
        telemetry.addLine("then copy 'Applied Power' → ShooterConfig.KS_INITIAL");
        telemetry.update();
    }

    private DcMotorEx initFlywheelMotor() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private double ticksPerSecondToRPM(double tps) {
        return tps * 60.0 / ShooterConfig.TICKS_PER_REV;
    }

    private void shutdown() {
        if (flywheelMotor != null) {
            flywheelMotor.setPower(0);
        }
    }
}
