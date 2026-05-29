package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * IndicatorSubsystem provides two distinct driver-feedback signals:
 *
 *   1. LED indicator (servo position): GREEN whenever a Limelight tag is in
 *      view. This matches what the original BlueTeleopv2 did and gives the
 *      driver an at-a-glance "the camera sees the goal" cue, even before
 *      the shooter has spun up.
 *
 *   2. Gamepad rumble: a one-shot pulse on the EDGE of becoming fully
 *      ready-to-fire (tag visible AND shooter at speed AND turret locked).
 *      This is the "go ahead and pull RT" cue.
 *
 * Edge-triggered to avoid rumble queue overflow.
 *
 * IMPORTANT: The gamepad reference is passed to update() on each call
 * rather than captured at construction. This is because in the FTC OpMode
 * lifecycle, gamepad1 and gamepad2 are NOT populated until after init()
 * runs — they're null during init. Capturing them at construction would
 * mean the rumble logic silently no-ops forever.
 */
public class IndicatorSubsystem {

    // Servo positions corresponding to LED colors on the indicator strip
    public static final double GREEN = 0.500;
    public static final double OFF   = 0.280;

    private final Servo indicator;     // may be null if not present

    private boolean wasReady = false;

    public IndicatorSubsystem(HardwareMap hardwareMap) {
        Servo ind = null;
        try {
            ind = hardwareMap.get(Servo.class, "indicator");
            ind.setPosition(OFF);
        } catch (Exception ignored) {
            // No indicator hardware on this robot.
        }
        indicator = ind;
    }

    /**
     * Update both indicator signals.
     *
     * @param tagVisible   true if Limelight has a target this frame
     * @param readyToFire  true if shooter is up AND turret is locked AND
     *                     tag is visible
     * @param gamepad      the gamepad to buzz on ready (typically gamepad2).
     *                     Pass null to skip rumble.
     */
    public void update(boolean tagVisible, boolean readyToFire, Gamepad gamepad) {
        // LED tracks tag visibility — useful as an "aim is good" cue
        if (indicator != null) {
            indicator.setPosition(tagVisible ? GREEN : OFF);
        }

        // Rumble is a one-shot edge trigger on full readiness
        if (gamepad != null) {
            if (readyToFire && !wasReady) {
                gamepad.rumble(0.8, 0.8, 250);
            } else if (!readyToFire && wasReady) {
                gamepad.stopRumble();
            }
        }
        wasReady = readyToFire;
    }

    public void off(Gamepad gamepad) {
        if (indicator != null) indicator.setPosition(OFF);
        if (gamepad != null) gamepad.stopRumble();
        wasReady = false;
    }
}
