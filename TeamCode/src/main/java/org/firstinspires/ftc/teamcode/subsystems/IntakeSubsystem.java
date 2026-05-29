package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * IntakeSubsystem wraps the two intake motors with operations that match
 * the physical ball path on this robot:
 *
 *   field ─[frontIntake]→ hopper ─[rearIntake]→ shooter
 *
 * Two distinct operations follow from that geometry:
 *
 *   1. {@link #intake()} — collect balls from the field. Both motors run
 *      forward: frontIntake grabs from the field while rearIntake makes
 *      room by feeding balls toward the shooter side of the hopper.
 *
 *   2. {@link #feedShooter()} — fire a staged ball. Only the REAR motor
 *      runs. Running the front during a shot would try to pull a new ball
 *      INTO the hopper while we're pushing one OUT, which can stall the
 *      front roller against a ball stuck in the floor of the hopper.
 *
 * Don't add an "indexer motor" alias — the rear intake IS the indexer for
 * this robot. The old ShooterConfig.INDEXER_MOTOR_NAME constant is a
 * relic from a different mechanical design and is no longer read by the
 * new code.
 */
public class IntakeSubsystem {

    public static final double FORWARD_POWER = 1.0;
    public static final double OUTTAKE_POWER = -1.0;

    private final DcMotor frontIntake;   // field -> hopper
    private final DcMotor rearIntake;    // hopper -> shooter

    public IntakeSubsystem(HardwareMap hardwareMap) {
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearIntake = hardwareMap.get(DcMotor.class, "rearIntake");
        rearIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Collect balls from the field. Both motors run forward. */
    public void intake() {
        frontIntake.setPower(FORWARD_POWER);
        rearIntake.setPower(FORWARD_POWER);
    }

    /** Reverse both motors to clear a jam or eject. */
    public void outtake() {
        frontIntake.setPower(OUTTAKE_POWER);
        rearIntake.setPower(OUTTAKE_POWER);
    }

    /**
     * Feed one ball into the shooter. Only the rear motor runs —
     * front is held off so a new ball can't try to enter while the
     * staged one is leaving.
     */
    public void feedShooter() {
        frontIntake.setPower(0);
        rearIntake.setPower(FORWARD_POWER);
    }

    /** Stop both motors. */
    public void stop() {
        frontIntake.setPower(0);
        rearIntake.setPower(0);
    }

    /**
     * Apply graduated power to BOTH motors (e.g. proportional outtake
     * via a trigger). For driver-controlled intake/outtake only — do
     * NOT use during firing.
     */
    public void setPower(double power) {
        frontIntake.setPower(power);
        rearIntake.setPower(power);
    }
}
