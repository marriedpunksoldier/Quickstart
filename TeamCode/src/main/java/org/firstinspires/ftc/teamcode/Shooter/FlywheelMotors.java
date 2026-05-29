package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * FlywheelMotors — shared two-motor flywheel handling for the tuning OpModes
 * (KSTune, KVTune, KPTune).
 *
 * The shooter uses two motors on opposite sides of the flywheel. The tuning
 * OpModes access motors directly (they deliberately bypass ShooterSubsystem
 * so they can apply raw open-loop power), so they need this helper to keep
 * the two-motor wiring details in one place.
 *
 * shooter1 — FORWARD; encoder used for velocity readings.
 * shooter2 — REVERSE; mechanically mirrored.
 *
 * If tuning only drove one motor, the KS/KV/KP values found would be wrong
 * for the real two-motor system (half the torque, different friction).
 * Always tune against both motors running.
 */
public class FlywheelMotors {

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    public FlywheelMotors(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR_1_NAME);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = hardwareMap.get(DcMotorEx.class, ShooterConfig.FLYWHEEL_MOTOR_2_NAME);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Apply the same power to both flywheel motors. */
    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    /** Signed RPM from motor 1's encoder (the control encoder). */
    public double getRPM() {
        return motor1.getVelocity() * 60.0 / ShooterConfig.TICKS_PER_REV;
    }

    /** Signed RPM from motor 2's encoder — health cross-check only. */
    public double getMotor2RPM() {
        return motor2.getVelocity() * 60.0 / ShooterConfig.TICKS_PER_REV;
    }

    /** Absolute RPM from motor 1 — convenient for tuning displays. */
    public double getAbsRPM() {
        return Math.abs(getRPM());
    }
}
